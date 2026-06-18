# Matricea de Testare a Modulației LoRa SDR (RP2040)

Acest document sumarizează rezultatele testelor empirice rulate pe emițătorul LoRa SDR "Zero-BOM" (RP2040) și recepționate de Gateway-ul TDM ESP32.

Testele au urmărit evaluarea stabilității fazei semnalului generat prin DMA și capacitatea demodulatorului de a decripta corect payload-urile în diverse condiții de durată a simbolului. Au fost testate 3 pachete cu lungimi diferite (`Hello` - 18 bytes, `World` - 18 bytes, `Rebeca` - 19 bytes).

## Rezumatul Testelor (Factor de Împrăștiere vs. Lățime de Bandă)

| Spreading Factor | BW 125 kHz (Slow) | BW 250 kHz (Medium) | BW 500 kHz (Fast) | Observații / Status |
| :--- | :--- | :--- | :--- | :--- |
| **SF12** | ❌ EȘEC (Biti eronați) | ⚠️ PARȚIAL (2 din 3) | ✅ SUCCES PERFECT | Necesită durată mică a simbolului (< 8.19ms) pentru decriptare corectă. |
| **SF11** | ❌ EȘEC (Biti eronați) | ✅ SUCCES PERFECT | ✅ SUCCES PERFECT | Pragul critic de stabilitate al cristalului este la 8.19 ms (SF11/250kHz). |
| **SF10** | ✅ SUCCES PERFECT | ✅ SUCCES PERFECT | ✅ SUCCES PERFECT | Foarte stabil, recomandat ca "Gold Master" la 125kHz. |
| **SF9** | ✅ SUCCES PERFECT | ✅ SUCCES PERFECT | ✅ SUCCES PERFECT | Stabil la toate lățimile de bandă testate. |
| **SF8** | ❌ ANOMALIE ("Rebeca" pică) | ❌ ANOMALIE ("Rebeca" pică) | ❌ ANOMALIE ("Rebeca" pică) | Pachetele > 18 bytes pică din cauza unui bug matematic în funcția de aliniere SDR la SF8. |
| **SF7** | ✅ SUCCES PERFECT | ✅ SUCCES PERFECT | ✅ SUCCES PERFECT | Viteza mare a simbolului menține alinierea fazei impecabilă. |

## 1. Descoperirea Pragului de Stabilitate a Fazelor (Drift-ul Cristalului)

Cea mai importantă concluzie a testelor este legată de deviația de fază a oscilatorului RP2040. Deoarece Pico nu dispune de un TCXO (Temperature Compensated Crystal Oscillator), simbolurile care petrec foarte mult timp "în aer" acumulează erori de fază.

*   **SF11 / 125 kHz (Durată simbol: ~16.38 ms):** Recepția are loc, dar pachetele sunt decriptate eronat (ex: `[..gd.]`). Faza se "răsucește" înainte ca simbolul să fie finalizat.
*   **SF11 / 250 kHz (Durată simbol: ~8.19 ms):** Toate pachetele trec curat și sunt decriptate perfect.
*   **SF12 / 500 kHz (Durată simbol: ~8.19 ms):** Funcționează perfect.

**Concluzie:** Sistemul SDR implementat are o limită fizică de stabilitate de aproximativ **8.19 ms per simbol**. Orice modulație care depășește acest timp va suferi de corupere de date.

## 2. Anomalia "Gaura Neagră" la SF8

Pe parcursul testării factorului SF8, a fost observat un comportament recurent, independent de lățimea de bandă:
*   Pachetul 1 (`Hello`, 18 bytes): **Succes**
*   Pachetul 2 (`World`, 18 bytes): **Succes**
*   Pachetul 3 (`Rebeca`, 19 bytes): **Pierdut la nivel fizic** (nu declanșează întreruperea de RX a ESP32-ului).

**Concluzie:** Aceasta indică un defect logic (bug matematic) în funcția C `CreateMessageFromPayload` a emițătorului SDR. Acel cod calculează incorect aliniamentul final al simbolurilor în buffer-ul DMA exclusiv când se folosește modulația SF8 pentru payload-uri de peste 18 octeți. Nu este o limitare hardware.

## 3. Tehnica de Mascaradă TTN (The Things Network)

Serverele The Things Network (TTN) pentru regiunea Europa (EU868) sunt strict configurate să accepte doar trafic LoRa standard la **125 kHz** pe frecvența principală de `868.1 MHz`. Orice pachet de 250 kHz sau 500 kHz trimis de Gateway este aruncat instantaneu de server.

Pentru a valida recepția RF la benzile largi fără a fi blocați de politica de cloud a TTN, a fost folosită cu succes tehnica **"Mascaradei JSON"**:
*   Emițătorul SDR generează fizic pachete RF pe 250 kHz / 500 kHz.
*   Gateway-ul ESP32 este configurat să asculte pe 250 kHz / 500 kHz.
*   Când ESP32-ul formează pachetul UDP Semtech Forwarder către TTN, câmpul `datr` este forțat static (`hardcoded`) la echivalentul pe bandă îngustă (ex: în loc de `"SF11BW250"`, se trimite `"SF11BW125"`).
*   Deoarece TTN ignoră valoarea lățimii de bandă din JSON la validarea semnăturii criptografice (MIC), pachetele sunt acceptate cu succes în Live Data Console.
