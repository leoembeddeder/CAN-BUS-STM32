# Matériel, câblage et procédure de test

## 1. Matériel

| Élément | Qté | Rôle |
|---|---|---|
| NUCLEO-G474RE | 2 | Nœuds CAN |
| NUCLEO-F429ZI | 1 | Nœud CAN |
| STM32 F476 | 2 | Nœuds CAN |
| MCP2551 | 1 par nœud | Transceiver CAN |
| Résistances 120 Ω | 2 | Terminaison du bus |
| PEAK-CAN USB | 1 | Analyse du bus avec PCAN-View |
| Analyseur logique Saleae | 1 | Analyse des signaux avec Logic 2 |
| Câbles | — | Bus CAN et liaisons UART / USB |

## 2. Câblage

### Bus CAN

```
 Nœud 1        Nœud 2        Nœud 3        Nœud 4        Nœud 5
[STM32]       [STM32]       [STM32]       [STM32]       [STM32]
  │ TX/RX       │ TX/RX       │ TX/RX       │ TX/RX       │ TX/RX
[MCP2551]     [MCP2551]     [MCP2551]     [MCP2551]     [MCP2551]
  │ H  L        │ H  L        │ H  L        │ H  L        │ H  L
──┴──┴──────────┴──┴──────────┴──┴──────────┴──┴──────────┴──┴──  CANH / CANL
120 Ω entre CANH et CANL                        120 Ω entre CANH et CANL
(extrémité gauche)                                (extrémité droite)
```

- Tous les CANH ensemble, tous les CANL ensemble.
- **Deux résistances de 120 Ω**, une à chaque extrémité (≈ 60 Ω mesurés entre CANH et CANL bus éteint).
- Masse commune entre les cartes.
- PEAK-CAN et analyseur Saleae se connectent **en dérivation** sur le bus.

### Liaison UART

Chaque nœud est relié à son PC par UART (LPUART1, 115 200 bauds). Chaque stagiaire dispose de son propre PC, sa carte et son dashboard.

> 📸 Photos et schémas de câblage : dossier [`../Images`](../Images).

## 3. Logiciels

| Logiciel | Usage |
|---|---|
| STM32CubeIDE | Configuration (CubeMX), compilation, flash, débogage |
| PyCharm / Python 3 | Dashboard PyQt5 |
| PCAN-View | Visualisation et analyse des trames CAN |
| Saleae Logic 2 | Analyse des signaux CAN et UART |

## 4. Procédure de mise en route

1. **Câbler** le bus CAN et vérifier la terminaison (≈ 60 Ω entre CANH et CANL, alimentation coupée).
2. **Compiler et flasher** le firmware sur chaque carte avec STM32CubeIDE.
3. **Vérifier le bus** : brancher PEAK-CAN, ouvrir PCAN-View à **250 kbit/s** et contrôler que les trames périodiques apparaissent (IDs et périodes attendus).
4. **Lancer le dashboard** :
   ```bash
   pip install pyserial PyQt5
   python can_dashboard.py
   ```
5. Sélectionner le port série, **115 200 bauds**, cliquer sur **CONNECT**.
6. Vérifier que les jauges évoluent et que le compteur de trames augmente.
7. **Tester les commandes** (ex. vitre droite) et contrôler la trame CAN sur PCAN-View et la trame UART sur Saleae.

## 5. Dépannage

| Symptôme | Piste |
|---|---|
| Aucune trame sur PCAN-View | Vérifier débit (250 kbit/s), terminaison 120 Ω, câblage CANH/CANL, alimentation du MCP2551 |
| Trames en erreur / bus-off | Débit ou horloge différents entre nœuds, terminaison absente ou double |
| Dashboard muet | Mauvais port COM ou baud ; contrôler avec Saleae la présence de `A5 …` sur la ligne TX |
| Trames UART décalées | Machine d'états : vérifier que le SOF `0xA5` est bien reçu en premier |
| Trame UART perdue | Transmission déjà en cours au moment de l'appel (émission en interruption) |

## 6. Validation

| Test | Outil | Résultat attendu |
|---|---|---|
| Débit et timing des bits | Saleae Logic 2 | 4 µs par bit (250 kbit/s) |
| Trames périodiques | PCAN-View | Bonne période, bons IDs et données |
| Retransmission CAN → UART | Saleae / dashboard | `A5 CMD DATA×4` cohérents avec la trame CAN |
| Commande UART → CAN | PCAN-View | Trame CAN émise après la commande |
| Réception par tous les nœuds | Dashboards | Chaque nœud voit les trames des 4 autres |
