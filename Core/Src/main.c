/**
 ******************************************************************************
 * @file    main.c
 * @brief   STM32 CAN Automotive Dashboard — Firmware
 *
 * Protocole UART (vers PC) :
 *   [ 0xA5 ][ CMD: 1 octet ][ DATA: 4 octets big-endian ] = 6 octets au total
 *
 * Protocole UART (depuis PC) :
 *   Même format 6 octets — la machine d'état HAL_UART_RxCpltCallback
 *   reconstruit les trames octet par octet.
 *
 * @note    Testé sur STM32G0 avec FDCAN à 250 kbps (HSI 40 MHz).
 ******************************************************************************
 */

#include "main.h"

/* ============================================================
 *  DÉFINITIONS DU PROTOCOLE
 *  Identiques des deux côtés (STM32 ↔ dashboard Python).
 * ============================================================ */

#define SOF_PATTERN   0xA5   /**< Start-Of-Frame — premier octet de chaque trame */
#define FRAME_LEN     6      /**< Longueur fixe d'une trame : SOF + CMD + 4 octets de data */

/* Commandes envoyées par le STM32 vers le PC (RX côté dashboard) */
#define CMD_CAN_406   0x30   /**< Trame CAN 0x406 — vitesse moteur (rpm)      */
#define CMD_CAN_55C   0x31   /**< Trame CAN 0x55C — pression (bar)             */
#define CMD_CAN_456   0x33   /**< Trame CAN 0x456 — température (°C)           */
#define CMD_CAN_679   0x88   /**< Trame CAN 0x679 — notification ABS           */
#define CMD_CAN_221   0x11   /**< Trame CAN 0x221 — batterie (V)    */
#define CMD_CAN_390   0x10   /**< Trame CAN 0x390 — débit air (g/s)            */
#define CMD_CAN_790   0x12   /**< Trame CAN 0x790 — notification porte droite  */
#define CMD_CAN_222   0x13   /**< Trame CAN 0x222 — notification external light  */
#define CMD_CAN_401   0x35   /**< Trame CAN 0x401 — notification vitre gauche  */
#define CMD_CAN_402   0x14/**< Trame CAN 0x402 — notification vitre droit  */


/* Commandes reçues par le STM32 depuis le PC (TX côté dashboard) */
#define CMD_PC_WINDOW_RIGHT  0x87   /**< PC → STM32 : actionner vitre droite  */
#define CMD_PC_WINDOW_LEFT   0x89   /**< PC → STM32 : actionner vitre gauche  */
#define CMD_PC_TRIGGER_ABS   0x90   /**< PC → STM32 : déclencher ABS (test)   */
#define CMD_PC_PORTE_GAUCHE  0x91   /**< PC → STM32 : ouvrir porte gauche     */

/* ============================================================
 *  TYPES
 * ============================================================ */

/**
 * @brief Structure d'une trame du protocole maison.
 *        Un octet de commande + 4 octets de payload.
 */
typedef struct {
    uint8_t cmd;
    uint8_t payload[4];
} FrameCom;

/**
 * @brief États de la machine d'état de réception UART octet par octet.
 */
typedef enum {
    WAIT_FOR_SOF,
    WAIT_FOR_CMD,
    WAIT_FOR_DATA1,
    WAIT_FOR_DATA2,
    WAIT_FOR_DATA3,
    WAIT_FOR_DATA4
} RxState;

/* ============================================================
 *  VARIABLES GLOBALES
 * ============================================================ */

FDCAN_HandleTypeDef hfdcan1;
UART_HandleTypeDef  hlpuart1;

/** Buffer de réception UART — 1 octet à la fois (mode IT) */
static uint8_t uart_rx_byte[1];

/** Trame en cours de reconstruction par la machine d'état RX */
static FrameCom rx_frame;

/** État courant de la machine d'état RX */
static RxState rx_state = WAIT_FOR_SOF;

/**
 * @brief Buffer d'émission UART persistant.
 * @note  CRITIQUE : HAL_UART_Transmit_IT ne copie pas le buffer —
 *        il doit rester valide jusqu'à la fin de la transmission.
 *        Ne jamais passer un buffer local à cette fonction.
 */
static uint8_t uart_tx_buffer[FRAME_LEN];

/** En-tête FDCAN pour les trames CAN sortantes */
static FDCAN_RxHeaderTypeDef fdcan_rx_header;

/** Buffer de données CAN reçues (8 octets max — CAN classique) */
static uint8_t fdcan_rx_data[8];

/* ============================================================
 *  PROTOTYPES
 * ============================================================ */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_LPUART1_UART_Init(void);

static uint8_t SendFrameUart(const FrameCom *frame);
static void    NewFrameReceivedCallback(const FrameCom *frame);

/* ============================================================
 *  POINT D'ENTRÉE
 * ============================================================ */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_LPUART1_UART_Init();

    BSP_LED_Init(LED_GREEN);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* ----------------------------------------------------------
     * Configuration du filtre FDCAN :
     * Accepte toutes les trames standard dans FIFO0
     * (mask = 0x000 → aucun bit filtré).
     * ---------------------------------------------------------- */
    FDCAN_FilterTypeDef filter = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000,
        .FilterID2    = 0x000,
    };
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
        Error_Handler();

    /* Rejeter les trames sans filtre correspondant */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);

    /* Activer l'interruption sur réception FIFO0 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        Error_Handler();

    /* Démarrer la réception UART en mode IT (1 octet à la fois) */
    HAL_UART_Receive_IT(&hlpuart1, uart_rx_byte, 1);

    /* ----------------------------------------------------------
     * Boucle principale :
     * Envoie périodiquement une trame de température simulée
     * (CAN ID 0x456) vers le dashboard PC toutes les 500 ms.
     * ---------------------------------------------------------- */
    FDCAN_TxHeaderTypeDef tx_header = {
        .Identifier          = 0x456,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };

    uint8_t  can_tx_data[8] = {0};
    uint8_t  counter        = 0;

    while (1)
    {
        /* Incrémenter le compteur de température simulée */
        can_tx_data[3] = counter++;

        /* Émettre la trame sur le bus CAN */
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, can_tx_data) != HAL_OK)
            Error_Handler();

        /* Relayer la trame vers le dashboard via UART */
        FrameCom frame = {
            .cmd        = CMD_CAN_456,
            .payload[0] = can_tx_data[0],
            .payload[1] = can_tx_data[1],
            .payload[2] = can_tx_data[2],
            .payload[3] = can_tx_data[3],
        };
        SendFrameUart(&frame);

        HAL_Delay(500);
    }
}

/* ============================================================
 *  CALLBACK FDCAN — RÉCEPTION CAN
 * ============================================================ */

/**
 * @brief  Appelé par le HAL à chaque trame CAN reçue dans FIFO0.
 *         Construit une FrameCom et la relaie vers le PC via UART.
 * @param  hfdcan   Handle FDCAN
 * @param  RxFifo0ITs  Flags d'interruption actifs
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
        return;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_rx_header, fdcan_rx_data) != HAL_OK)
        return;

    FrameCom frame = {
        .payload[0] = fdcan_rx_data[0],
        .payload[1] = fdcan_rx_data[1],
        .payload[2] = fdcan_rx_data[2],
        .payload[3] = fdcan_rx_data[3],
    };

    /* Associer chaque ID CAN à sa commande UART */
    switch (fdcan_rx_header.Identifier)
    {
        case 0x406: frame.cmd = CMD_CAN_406; break; //rpm
        case 0x55C: frame.cmd = CMD_CAN_55C; break; //pression
        case 0x679: frame.cmd = CMD_CAN_679; break; //notification abs
        case 0x390: frame.cmd = CMD_CAN_390; break; //débit d'air
        case 0x221: frame.cmd = CMD_CAN_221; break; //batterie
        case 0x790: frame.cmd = CMD_CAN_790; break; //notification porte droite
        case 0x222: frame.cmd = CMD_CAN_222; break; //notification external light
        case 0x401: frame.cmd = CMD_CAN_401; break; //notification vitre gauche

        default:    return; /* ID inconnu — ignorer */
    }

    SendFrameUart(&frame);
}

/* ============================================================
 *  ENVOI UART → PC
 * ============================================================ */

/**
 * @brief  Sérialise une FrameCom et l'envoie via UART en mode IT.
 *
 * @note   Le buffer uart_tx_buffer est global pour rester valide
 *         pendant toute la durée de la transmission IT.
 *         Appel depuis une IT CAN : si une transmission est déjà
 *         en cours, la trame est perdue. Pour un projet production,
 *         utiliser une file circulaire de trames.
 *
 * @param  frame  Pointeur vers la trame à envoyer
 * @retval 0 si succès, 1 si erreur HAL
 */
static uint8_t SendFrameUart(const FrameCom *frame)
{
    uart_tx_buffer[0] = SOF_PATTERN;
    uart_tx_buffer[1] = frame->cmd;
    uart_tx_buffer[2] = frame->payload[0];
    uart_tx_buffer[3] = frame->payload[1];
    uart_tx_buffer[4] = frame->payload[2];
    uart_tx_buffer[5] = frame->payload[3];

    return (HAL_UART_Transmit_IT(&hlpuart1, uart_tx_buffer, FRAME_LEN) != HAL_OK) ? 1 : 0;
}

/* ============================================================
 *  RÉCEPTION UART — MACHINE D'ÉTAT OCTET PAR OCTET
 * ============================================================ */

/**
 * @brief  Appelé par le HAL après réception de chaque octet UART.
 *         Reconstruit les trames de 6 octets selon le protocole.
 * @param  huart  Handle UART ayant déclenché l'interruption
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t byte = uart_rx_byte[0];

    switch (rx_state)
    {
        case WAIT_FOR_SOF:
            if (byte == SOF_PATTERN)
                rx_state = WAIT_FOR_CMD;
            break;

        case WAIT_FOR_CMD:
            rx_frame.cmd = byte;
            rx_state = WAIT_FOR_DATA1;
            break;

        case WAIT_FOR_DATA1:
            rx_frame.payload[0] = byte;
            rx_state = WAIT_FOR_DATA2;
            break;

        case WAIT_FOR_DATA2:
            rx_frame.payload[1] = byte;
            rx_state = WAIT_FOR_DATA3;
            break;

        case WAIT_FOR_DATA3:
            rx_frame.payload[2] = byte;
            rx_state = WAIT_FOR_DATA4;
            break;

        case WAIT_FOR_DATA4:
            rx_frame.payload[3] = byte;
            NewFrameReceivedCallback(&rx_frame);
            rx_state = WAIT_FOR_SOF;
            break;
    }

    /* Réarmer la réception pour le prochain octet */
    HAL_UART_Receive_IT(&hlpuart1, uart_rx_byte, 1);
}

/* ============================================================
 *  TRAITEMENT DES COMMANDES REÇUES DEPUIS LE PC
 * ============================================================ */

/**
 * @brief  Traite une trame complète reçue depuis le dashboard PC.
 *         Chaque commande peut déclencher une action sur le bus CAN
 *         ou sur les périphériques locaux (GPIO, etc.).
 * @param  frame  Trame reçue et décodée
 */
static void NewFrameReceivedCallback(const FrameCom *frame)
{
    FDCAN_TxHeaderTypeDef tx_header = {
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };

    uint8_t tx_data[8] = {0};

    switch (frame->cmd)
    {
        case CMD_PC_WINDOW_RIGHT:
            /* Envoyer une commande CAN pour actionner la vitre droite */
            tx_header.Identifier = 0x402;
            tx_data[0] = 0x01;
            tx_data[1] = 0x02;
            tx_data[2] = 0x03;
            tx_data[3] = 0x04;
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK)
                Error_Handler();
            /* Relayer la trame vers le dashboard via UART */
            FrameCom frame = {
                .cmd        = CMD_CAN_402,
                .payload[0] = tx_data[0],
                .payload[1] = tx_data[1],
                .payload[2] = tx_data[2],
                .payload[3] = tx_data[3],
            };
            SendFrameUart(&frame);
            break;

        case CMD_PC_PORTE_GAUCHE:
            /* Réservé — traitement porte gauche à implémenter */
            break;

        case CMD_PC_TRIGGER_ABS:
            /* Réservé — simulation ABS à implémenter */
            break;

        default:
            /* Commande inconnue — ignorer silencieusement */
            break;
    }
}

/* ============================================================
 *  CONFIGURATION HORLOGE ET PÉRIPHÉRIQUES (généré par CubeMX)
 * ============================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* HSI → PLL → SYSCLK 80 MHz */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM           = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN           = 10;
    RCC_OscInitStruct.PLL.PLLP           = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ           = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR           = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/**
 * @brief  Initialisation FDCAN1.
 *         250 kbps en CAN classique — HSI 40 MHz / prescaler 10 / 16 TQ.
 *         Point d'échantillonnage à ~87.5 %.
 */
static void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = DISABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalPrescaler     = 10;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1      = 13;
    hfdcan1.Init.NominalTimeSeg2      = 2;
    hfdcan1.Init.DataPrescaler        = 10;
    hfdcan1.Init.DataSyncJumpWidth    = 1;
    hfdcan1.Init.DataTimeSeg1         = 13;
    hfdcan1.Init.DataTimeSeg2         = 2;
    hfdcan1.Init.StdFiltersNbr        = 1;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
        Error_Handler();
}

/**
 * @brief  Initialisation LPUART1.
 *         115 200 baud, 8N1, mode IT, sans contrôle de flux.
 */
static void MX_LPUART1_UART_Init(void)
{
    hlpuart1.Instance            = LPUART1;
    hlpuart1.Init.BaudRate       = 115200;
    hlpuart1.Init.WordLength     = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits       = UART_STOPBITS_1;
    hlpuart1.Init.Parity         = UART_PARITY_NONE;
    hlpuart1.Init.Mode           = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&hlpuart1) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
        Error_Handler();
}

/** @brief  Activation des horloges GPIO. */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* ============================================================
 *  GESTIONNAIRE D'ERREUR
 * ============================================================ */

/**
 * @brief  Appelé en cas d'erreur HAL irrécupérable.
 *         Désactive les IRQ et boucle indéfiniment.
 *         En production : allumer une LED d'erreur ou déclencher le watchdog.
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* Ajouter ici un printf ou un log d'erreur si besoin */
    (void)file;
    (void)line;
}
#endif
