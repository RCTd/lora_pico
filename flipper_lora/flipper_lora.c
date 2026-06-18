#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <gui/view_port.h>
#include <input/input.h>
#include <stm32wbxx_ll_spi.h>
#include <stm32wbxx_ll_dma.h>
#include <stm32wbxx_ll_gpio.h>
#include <stm32wbxx_ll_bus.h>
#include <math.h>

#define LORAWAN 1
#include "lolra_lib/LoRa-SDR-Code.h"
#include "lolra_lib/lorawan_simple.h"

// Setări LoRa (SF7)
#define CHIRP_WORDS 16384 // ~8.192 ms la 32 Mbps (16-bit SPI)
#define SYMBOL_SHIFT_WORDS (CHIRP_WORDS / 128) // 128 words per shift

uint16_t up_chirp_ram[CHIRP_WORDS];

void precalculate_chirps() {
    uint32_t phase_up = 0;
    // Matematica pentru 865.1 MHz la 32 MHz Sample Rate
    uint32_t phase_inc_up = 139250893; // Baza: 1.0375 MHz
    uint32_t phase_inc_inc = 64;       // Sweep (Chirp Rate)
    
    for(int i = 0; i < CHIRP_WORDS; i++) {
        uint16_t word_up = 0;
        for(int b = 0; b < 16; b++) {
            phase_up += phase_inc_up;
            phase_inc_up += phase_inc_inc;
            if(phase_up & 0x80000000) word_up |= (1 << (15-b));
        }
        up_chirp_ram[i] = word_up;
    }
}

// Oprim generarea complexă de pachete pentru a retesta semnalul de bază.
// Vom trimite doar 10 up-chirps brute, exact cum funcționa înainte!

static void input_callback(InputEvent* event, void* ctx) {
    furi_message_queue_put((FuriMessageQueue*)ctx, event, FuriWaitForever);
}

int32_t flipper_lora_main(void* p) {
    UNUSED(p);
    FURI_LOG_E("LoRaTX", "Aplicație LoRa Pornită! Calculăm formele de undă...");
    precalculate_chirps();

    FuriMessageQueue* event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    ViewPort* view_port = view_port_alloc();
    view_port_input_callback_set(view_port, input_callback, event_queue);
    Gui* gui = (Gui*)furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_external);
    
    LL_SPI_Disable(SPI1);
    LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
    LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);
    LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV2);
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_SPI_Enable(SPI1);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2 | LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_7, LL_DMAMUX_REQ_SPI1_TX);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_HALFWORD);

    bool app_running = true;
    InputEvent event;
    while(app_running) {
        FURI_LOG_E("LoRaTX", "Transmitem 10 up-chirps brute...");
        
        for(int c = 0; c < 10 && app_running; c++) {
            LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_7);
            LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_7, (uint32_t)up_chirp_ram, (uint32_t)&(SPI1->DR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
            LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_7, CHIRP_WORDS);
            LL_DMA_ClearFlag_TC7(DMA2);
            LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_7);
            
            int timeout = 20; 
            while(!LL_DMA_IsActiveFlag_TC7(DMA2) && app_running && timeout > 0) {
                furi_delay_ms(1);
                timeout--;
            }
            
            furi_delay_ms(1); // 1 ms gap between the 10 chirps
        }
        
        LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_7);
        
        FURI_LOG_E("LoRaTX", "Packet Trimis! Pauză 2 secunde...");
        // Așteptăm 2 secunde și verificăm butoanele
        for(int i = 0; i < 200 && app_running; i++) {
            if(furi_message_queue_get(event_queue, &event, 10) == FuriStatusOk) { 
                if(event.type == InputTypeShort && event.key == InputKeyBack) {
                    app_running = false;
                }
            }
        }
    }

    LL_SPI_Disable(SPI1);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_external);

    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_message_queue_free(event_queue);
    furi_record_close(RECORD_GUI);

    return 0;
}
