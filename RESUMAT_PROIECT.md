# Sistem LoRaWAN "Zero-BOM" cu Emițător SDR RP2040 și Gateway TDM ESP32

Acest document descrie arhitectura, implementarea și fluxul operațional al ecosistemului LoRaWAN dezvoltat, care integrează un emițător radio software (SDR) pe Raspberry Pi Pico și un gateway inteligent pe ESP32.

## 1. Arhitectura Sistemului

Sistemul este compus din trei niveluri:
1.  **Nodul Senzor (Pico SDR):** Generează semnale LoRa fizice direct din pinii digitali (PIO) folosind tehnici SDR, fără cip radio dedicat.
2.  **Gateway (ESP32 Zephyr RTOS):** Recepționează semnalele, le decriptează local pentru debug și le redirecționează către Cloud prin Wi-Fi folosind un protocol de multiplexare în timp (TDM).
3.  **Cloud (The Things Network):** Primește datele, le validează criptografic și le afișează în consola de utilizator.

---

## 2. Structura Directoarelor

*   `/nas/Pico/pico/pico-examples/` - Directorul rădăcină al mediului de dezvoltare.
*   `lora_pico/` - Repository-ul principal al proiectului.
    *   `multicore.c` - Codul sursă pentru Raspberry Pi Pico (logică SDR + LoRaWAN ABP + Flash Persistence).
    *   `esp32_lora/` - Aplicația Zephyr RTOS pentru Gateway.
        *   `src/main.c` - Logica TDM, Wi-Fi, Redirecționare UDP și Store-and-Forward.
        *   `prj.conf` - Configurația kernel-ului Zephyr (MBEDTLS, NVS, Wi-Fi, Display).
    *   `lolra/` - Biblioteca de bază pentru sinteza pachetelor LoRaWAN pe Pico.
*   `zephyrproject/` - Mediul Zephyr SDK și uneltele necesare compilării pentru ESP32.

---

## 3. Fluxul de Lucru și Comenzi

### A. Programarea Nodului Raspberry Pi Pico
Pico rulează un motor SDR pe Nucleul 1 și logica de salvare a contorului în Flash pe Nucleul 0.

1.  **Navigare:** `cd /nas/Pico/pico/pico-examples/build_pico_w`
2.  **Compilare:** `make lora_dma -j4`
3.  **Flash (via picotool):** `picotool load -f lora_pico/lora_dma.elf -x`

### B. Programarea Gateway-ului ESP32 (T-Beam)
Gateway-ul rulează Zephyr RTOS cu un sistem avansat de gestionare a memoriei.

1.  **Activare Mediu:** `export PATH="/nas/Pico/pico/pico-examples/zephyrproject/.venv/bin:$PATH"`
2.  **Compilare:** 
    ```bash
    west build -b ttgo_t_beam/esp32/procpu -p always lora_pico/esp32_lora
    ```
3.  **Flash:**
    ```bash
    west flash --esp-device /dev/ttyACM1 --esp-baud-rate 115200
    ```
4.  **Monitorizare (fără reset):**
    ```bash
    python3 read_esp_noreset.py --port /dev/ttyACM1
    ```

---

## 4. Realizări Tehnice Cheie

### 1. Sinteza Radio Software (Pico SDR)
*   S-a obținut o transmisie stabilă la **864.92 MHz** (armonica a 36-a a ceasului PIO).
*   Implementare **LoRaWAN ABP** completă cu AES-128 (MIC + Payload Encryption).
*   **Flash Persistence:** Salvarea `Frame Counter`-ului în memoria nevolatilă a Pico pentru a preveni respingerea pachetelor de către TTN la resetare.

### 2. TDM Shielding & Wi-Fi Coexistence (ESP32)
*   S-a rezolvat problema zgomotului RF produs de Wi-Fi care "orbea" receptorul LoRa.
*   **Logica TDM:** Radioul SX1276 este pus în IDLE forțat în timpul tranzacțiilor Wi-Fi folosind mutex-uri kernel.

### 3. Mecanismul Store-and-Forward (Flash NVS)
*   Dacă Wi-Fi-ul cade, Gateway-ul salvează pachetele în partiția `storage` a Flash-ului ESP32.
*   Pachetele rețin **timestamp-ul original** (în microsecunde).
*   La restabilirea conexiunii, coada de așteptare este trimisă automat către TTN ("Draining").

### 4. Integrarea Cloud (TTN Masquerade)
*   Gateway-ul raportează frecvența standard de **868.1 MHz** în JSON-ul Semtech UDP, deși fizic primește pe 864.92 MHz.
*   S-au implementat **micro-timestamps** pentru conformitatea cu protocolul Semtech V2.

---

## 5. Setări Critice în Consola TTN
Pentru funcționarea corectă, dispozitivul trebuie configurat în modul **ABP** cu următoarele setări:
*   **Resets Frame Counters:** Activat (Enabled) - esențial pentru faza de prototipare.
*   **DevAddr / NwkSKey / AppSKey:** Trebuie să coincidă cu valorile din `multicore.c` și `main.c`.
*   **Payload Formatter:** Custom Javascript pentru conversia Hex -> ASCII.

---

## 6. Documentație și Redactarea Tezei (Overleaf & Git)

Documentația proiectului și Teza de Master sunt gestionate printr-un sistem avansat de control al versiunilor și compilare LaTeX.

### Arhitectura Documentației
*   `/nas/Pico/pico/pico-examples/Teza_Master/` - Directorul care conține fișierele LaTeX și resursele grafice ale tezei. Acesta funcționează ca un bridge între mediul local și instanța Overleaf.
*   `/nas/Pico/pico/pico-examples/lora_pico/` - Repository-ul Git principal pentru codul sursă.
*   `OVERLEAF_LATEX_CONTAINER_SETUP_AND_PATCHES.md` - Documentul care detaliază setările instanței locale de Overleaf, inclusiv patch-urile de autentificare (`Bearer` token) și fix-urile pentru baza de date MongoDB.

### Utilizarea Containerului Overleaf Local
Teza este redactată folosind o instanță privată de Overleaf (Docker container), îmbunătățită cu următoarele capabilități:
1.  **Git Bridge:** Permite sincronizarea bidirecțională între Overleaf și un repository Git local. Autentificarea se face cu utilizatorul `git` și un Personal Access Token (PAT) generat în setările utilizatorului Overleaf.
2.  **LaTeX Extins:** Containerul include pachete complete LaTeX (`babel-romanian`, `minted`, `todonotes`, etc.) și suportă flag-ul `--shell-escape` (activat prin `latexmkrc`), necesar pentru generarea automată a evidențierii de sintaxă în codul C/Python inclus în lucrare.
3.  **Flux de Lucru:** 
    *   Teza este scrisă în browser via UI-ul Overleaf local (port `8082`).
    *   Din linia de comandă (directorul `/nas/Pico/pico/pico-examples/Teza_Master/`), se execută `git pull` / `git push` folosind URL-ul bridge-ului preconfigurat: 
        `http://git:olp_XYHQJeyZy5XVq3juBsA7uiISNTcUYJZBOwKK@nas-pi.local:8082/git/6a201df2729464b3c36f5ae6`

---
*Document creat automat de Gemini CLI Agent ca rezumat final al proiectului de Master.*
