# Configuration CAN (FDCAN utilisé en CAN classique)

Tous les nœuds du bus partagent **exactement la même configuration**.

## 1. Synthèse

| Paramètre | Valeur |
|---|---|
| Périphérique | FDCAN1 |
| Format de trame | CAN classique (`FDCAN_FRAME_CLASSIC`) |
| Mode | Normal (`FDCAN_MODE_NORMAL`) |
| Identifiants | Standard 11 bits |
| Débit nominal | 250 kbit/s |
| Filtrage | Aucun : toutes les trames sont reçues (FIFO0) |
| Retransmission automatique | Désactivée |
| Mode FIFO d'émission | `FDCAN_TX_FIFO_OPERATION` |
| Horloge | HSI → PLL → SYSCLK 80 MHz ; horloge FDCAN 40 MHz |

## 2. Horloge

- Source : **oscillateur interne HSI** (16 MHz), passant par la **PLL** (M = 1, N = 10, R = 2) pour obtenir un SYSCLK de 80 MHz.
- Bus APB1 : 40 MHz, utilisé comme horloge du FDCAN.
- Le choix de la HSI (plutôt que la HSE) a été fait après un problème de stabilité rencontré avec l'horloge externe. Il est appliqué de façon **identique sur tous les nœuds**.

## 3. Code d'initialisation

```c
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
```

## 4. Timing binaire

### 4.1 Découpage d'un bit

Un bit CAN est divisé en **quanta de temps (TQ)** :

```
|<- Sync_Seg ->|<------- TimeSeg1 ------->|<--- TimeSeg2 --->|
|     1 TQ     |          13 TQ           |       2 TQ       |
                                           ^
                                  point d'échantillonnage
```

- **Sync_Seg** (1 TQ, fixe) : les fronts attendus du signal doivent survenir ici.
- **TimeSeg1** : regroupe le segment de propagation et le premier segment de phase. Le point d'échantillonnage se situe à la fin de ce segment.
- **TimeSeg2** : deuxième segment de phase, après le point d'échantillonnage.

### 4.2 Calcul du débit

```
f_FDCAN   = 40 MHz
TQ        = NominalPrescaler / f_FDCAN = 10 / 40 MHz = 250 ns
Bit       = 1 + NominalTimeSeg1 + NominalTimeSeg2 = 1 + 13 + 2 = 16 TQ
Durée bit = 16 × 250 ns = 4 µs
Débit     = 1 / 4 µs = 250 kbit/s
```

### 4.3 Point d'échantillonnage

```
SP = (1 + TimeSeg1) / (1 + TimeSeg1 + TimeSeg2) = 14 / 16 = 87,5 %
```

Un point d'échantillonnage vers 87,5 % est une valeur courante pour le CAN : il laisse le maximum de temps au signal pour se stabiliser sur le bus avant la lecture du bit, tout en gardant 2 TQ pour la resynchronisation.

## 5. Les paramètres clés en détail

### `NominalTimeSeg1 = 13`

Nombre de TQ **avant** le point d'échantillonnage. Il compense le délai de propagation sur le bus et dans les transceivers (MCP2551) et laisse le temps au signal de se stabiliser. Une valeur plus grande déplace le point d'échantillonnage vers la fin du bit.

### `NominalTimeSeg2 = 2`

Nombre de TQ **après** le point d'échantillonnage. Il fixe la marge restante avant le bit suivant et limite la plage de resynchronisation : **SJW ≤ TimeSeg2**.

### `NominalSyncJumpWidth = 1`

Amplitude maximale (en TQ) dont le contrôleur peut **allonger TimeSeg1 ou raccourcir TimeSeg2** pour se recaler sur les fronts reçus (resynchronisation), afin de compenser les petites différences de fréquence entre nœuds.

- Valeur 1 = la plus petite : suffisante quand tous les nœuds utilisent la même source d'horloge (HSI + PLL) et que le bus est court.
- À augmenter (jusqu'à TimeSeg2) si l'on rencontre des erreurs de synchronisation sur un bus plus long ou avec des horloges moins précises.

### Paramètres `Data*`

| Paramètre | Valeur |
|---|---|
| `DataPrescaler` | 10 |
| `DataSyncJumpWidth` | 1 |
| `DataTimeSeg1` | 13 |
| `DataTimeSeg2` | 2 |

Ces paramètres décrivent la **phase de données du CAN FD** lorsque le *Bit Rate Switch* est actif. En CAN classique (`FDCAN_FRAME_CLASSIC`, `BitRateSwitch = OFF`), ils sont **sans effet**. Ils sont renseignés avec les mêmes valeurs que la phase nominale pour garder une configuration cohérente et faciliter une migration vers CAN FD.

## 6. Filtrage et réception

```c
FDCAN_FilterTypeDef filter = {
    .IdType       = FDCAN_STANDARD_ID,
    .FilterIndex  = 0,
    .FilterType   = FDCAN_FILTER_MASK,
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
    .FilterID1    = 0x000,
    .FilterID2    = 0x000,
};
```

- Filtre en mode masque avec `ID = 0x000` et `MASK = 0x000` : **aucun bit n'est comparé**, donc toutes les trames standard sont acceptées dans la FIFO0.
- Filtre global : trames non correspondantes rejetées, trames distantes (RTR) rejetées.
- Notification activée : `FDCAN_IT_RX_FIFO0_NEW_MESSAGE` → le callback `HAL_FDCAN_RxFifo0Callback` est appelé à chaque trame reçue.

## 7. Émission

Les trames sont émises avec `HAL_FDCAN_AddMessageToTxFifoQ`, avec l'en-tête suivant :

| Champ | Valeur |
|---|---|
| `IdType` | `FDCAN_STANDARD_ID` |
| `TxFrameType` | `FDCAN_DATA_FRAME` |
| `DataLength` | `FDCAN_DLC_BYTES_8` |
| `BitRateSwitch` | `FDCAN_BRS_OFF` |
| `FDFormat` | `FDCAN_CLASSIC_CAN` |
| `TxEventFifoControl` | `FDCAN_NO_TX_EVENTS` |

## 8. Couche physique

- Transceiver **MCP2551** sur chaque nœud (broches TX/RX reliées au FDCAN1).
- Bus 2 fils CANH / CANL.
- **Résistance de terminaison de 120 Ω à chaque extrémité** du bus (soit environ 60 Ω mesurés entre CANH et CANL).
