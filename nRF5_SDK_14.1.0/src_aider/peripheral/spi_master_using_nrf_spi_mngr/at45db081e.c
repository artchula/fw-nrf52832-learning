#include "at45db081e.h"



int8_t at45db_init(at45db_dev_t *dev) {
        uint8_t i = 0, id = 0;
         /* Enable SPI SPI_MODE0*/
         Init_SPI1_location2();
              U1GCR |=  U1GCR_CPOL; //set SCK high when idle
       //        U1GCR |=  U1GCR_CPHA;
              P1SEL &= ~(BIT3);
              P1DIR |=  (BIT3);       // CS pin as output
//              P1SEL &= ~(BIT7);
//              P1DIR &=  ~(BIT7);      
//              P2    |=  (BIT0);       //de-select spi flash  
              CS_H(); 
             
        /*setup buffer manager to perform write and read operations*/
        buffer_mgr.active_buffer = 0;
        buffer_mgr.buffer_addr[0] = AT45DB_BUFFER_1;
        buffer_mgr.buffer_addr[1] = AT45DB_BUFFER_2;
        buffer_mgr.buf_to_page_addr[0] = AT45DB_BUF_1_TO_PAGE;
        buffer_mgr.buf_to_page_addr[1] = AT45DB_BUF_2_TO_PAGE;
//        mspi_chip_release(AT45DB_CS);
        /*init mspi in mode3, at chip select pin 3 and max baud rate*/
//        mspi_init(AT45DB_CS, MSPI_MODE_3, MSPI_BAUD_MAX);
    
        while ((id&128)==0) {
//            uart0_printf("\n id:");
          //  uart0_printDEC(id);
        ///   uart0_printf("\n");
          LED1_BSP ^= 1;
//                mspi_chip_select(AT45DB_CS);
                CS_L();
                 delay_ms(10);
//                spiGetByte1(0x9F);
                id = spiGetByte1(0xD7);
//                id = spiGetByte1(0x00);

                //delay_ms(1);
                 delay_ms(10);
                CS_H(); 
                delay_ms(200);
//                mspi_chip_release(AT45DB_CS);
               
//                if (i++ > 250) {
//                        return -1;
//                }
        }
        uart0_printf("\n Finish.");
        return 1;
}
void at45db_erase_chip(void) {
        /*chip erase command consists of 4 byte*/
        uint8_t cmd[4] = { 0xC7, 0x94, 0x80, 0x9A };
        at45db_write_cmd(&cmd[0]);
//        mspi_chip_release(AT45DB_CS);
        CS_H();
        /*wait until AT45DB161 is ready again*/
//        mspi_chip_select(AT45DB_CS);
        CS_L(); 
        at45db_busy_wait();
//        mspi_chip_release(AT45DB_CS);
        CS_H();
}
//void at45db_erase_block(uint16_t addr) {
//        /*block erase command consists of 4 byte*/
//        uint8_t cmd[4] = { AT45DB_BLOCK_ERASE, (uint8_t) (addr >> 3),
//                        (uint8_t) (addr << 5), 0x00 };
//        at45db_write_cmd(&cmd[0]);
////        mspi_chip_release(AT45DB_CS);
//        CS_H();
//        /*wait until AT45DB161 is ready again*/
////        mspi_chip_select(AT45DB_CS);
//        CS_L(); 
//        at45db_busy_wait();
////        mspi_chip_release(AT45DB_CS);
//        CS_H(); 
//}
//void at45db_erase_page(uint16_t addr) {
//        /*block erase command consists of 4 byte*/
//        uint8_t cmd[4] = { AT45DB_PAGE_ERASE, (uint8_t) (addr >> 6),
//                        (uint8_t) (addr << 2), 0x00 };
//        at45db_write_cmd(&cmd[0]);
////        mspi_chip_release(AT45DB_CS);
//       CS_H();   
//        /*wait until AT45DB161 is ready again*/
////        mspi_chip_select(AT45DB_CS);
//       CS_L(); 
//        at45db_busy_wait();
////        mspi_chip_release(AT45DB_CS);
//       CS_H();  
//}
//void at45db_write_buffer(uint16_t addr, uint8_t *buffer, uint16_t bytes) {
//  
//       
//  
//        uint16_t i;
//        /*block erase command consists of 4 byte*/
//        CS_L(); 
//        uint8_t cmd[4] = { buffer_mgr.buffer_addr[buffer_mgr.active_buffer], 0x00,(uint8_t) (addr >> 8), (uint8_t) (addr) };
//       
//        
//        
//        at45db_write_cmd(&cmd[0]);     
//        
//        
//        
//        for (i = 0; i < bytes; i++) {
////                mspi_transceive(*buffer++);
//                  spiSendByte(*buffer++);  
//        }
//       
////        mspi_chip_release(AT45DB_CS);
//        delay_us(10);
//        CS_H();  
//         
//}
//void at45db_buffer_to_page(uint16_t addr) {
//        /*wait until AT45DB161 is ready again*/
//        at45db_busy_wait();
//        /*write active buffer to page command consists of 4 byte*/
//        uint8_t cmd[4] = { buffer_mgr.buf_to_page_addr[buffer_mgr.active_buffer],
//                        (uint8_t) (addr >> 6), (uint8_t) (addr << 2), 0x00 };
//        at45db_write_cmd(&cmd[0]);
////        mspi_chip_release(AT45DB_CS);
//        CS_H();  
//        /* switch active buffer to allow the other one to be written,
//         * while these buffer is copied to the Flash EEPROM page*/
//        buffer_mgr.active_buffer ^= 1;
//}
//void at45db_read_page_buffered(uint16_t p_addr, uint16_t b_addr,
//                uint8_t *buffer, uint16_t bytes) {
//        /*wait until AT45DB161 is ready again*/
//        at45db_busy_wait();
//        at45db_page_to_buf(p_addr);
//        at45db_read_buffer(b_addr, buffer, bytes);
//}
//void at45db_read_page_bypassed(uint16_t p_addr, uint16_t b_addr,
//                uint8_t *buffer, uint16_t bytes) {
//        uint16_t i;
//        /*wait until AT45DB161 is ready again*/
//        at45db_busy_wait();
//        /* read bytes directly from page command consists of 4 cmd bytes and
//         * 4 don't care */
//        uint8_t cmd[4] = { AT45DB_PAGE_READ, (uint8_t) (p_addr >> 6),
//                        (((uint8_t) (p_addr << 2)) & 0xFC) | ((uint8_t) (b_addr >> 8)),
//                        (uint8_t) (b_addr) };
//        at45db_write_cmd(&cmd[0]);
//        for (i = 0; i < 4; i++) {
////                mspi_transceive(0x00);
//                  spiSendByte(0x00);
//        }
//        /*now the data bytes can be received*/
//        for (i = 0; i < bytes; i++) {
////                *buffer++ = mspi_transceive(MSPI_DUMMY_BYTE);
//                  *buffer++ =  spiGetByte1(MSPI_DUMMY_BYTE);
//        }
////        mspi_chip_release(AT45DB_CS);
//         CS_H();  
//}
//void at45db_page_to_buf(uint16_t addr) {
//        /*write active buffer to page command consists of 4 byte*/
//        uint8_t cmd[4] = { AT45DB_PAGE_TO_BUF, (uint8_t) (addr >> 6),
//                        (uint8_t) (addr << 2), 0x00 };
//        at45db_write_cmd(&cmd[0]);
////        mspi_chip_release(AT45DB_CS);
//        CS_H();  
//        /* switch active buffer to allow the other one to be written,
//         * while these buffer is copied to the Flash EEPROM page*/
//        //buffer_mgr.active_buffer ^= 1;
//        at45db_busy_wait();
//}
//void at45db_read_buffer(uint8_t b_addr, uint8_t *buffer, uint16_t bytes) {
//        uint16_t i;
//        uint8_t cmd[4] = { AT45DB_READ_BUFFER, 0x00, (uint8_t) (b_addr >> 8),(uint8_t) (b_addr) };
//        at45db_busy_wait();
//        at45db_write_cmd(&cmd[0]);
////        mspi_transceive(0x00);
//        spiSendByte(0x00);
//        for (i = 0; i < bytes; i++) {
////                *buffer++ = mspi_transceive(0x00);
//          *buffer++ = spiGetByte1(0x00);
//        }
////        mspi_chip_release(AT45DB_CS);
//         CS_H();  
//}
//void at45db_write_cmd(uint8_t *cmd) {
//        uint8_t i;
////        mspi_chip_select(AT45DB_CS);
//         CS_L();  
//         for (i = 0; i < 4; i++) {
////                mspi_transceive(*cmd++);
//          spiSendByte(*cmd++);
//        }
//       
//}
void at45db_busy_wait(void) {
//        mspi_chip_select(AT45DB_CS);
        CS_L();  
//        mspi_transceive(AT45DB_STATUS_REG);
        spiSendByte(AT45DB_STATUS_REG);
//        while ((mspi_transceive(MSPI_DUMMY_BYTE) >> 7) != 0x01) {
        while ((spiGetByte1(MSPI_DUMMY_BYTE) >> 7) != 0x01) {
//        mspi_chip_release(AT45DB_CS);
          CS_H();  
        }
}

//void TEST_CALL(const uint16_t addr,const uint16_t bytes,uint8_t *buffer)//uint16_t addr, uint8_t *buffer, uint16_t bytes)
//{
//        uint16_t i;
//        /*block erase command consists of 4 byte*/
//        CS_L(); 
//        uint8_t cmd[4] = { buffer_mgr.buffer_addr[buffer_mgr.active_buffer], 0x00,(uint8_t) (addr >> 8), (uint8_t) (addr) };
//       
//        
//        
//        at45db_write_cmd(&cmd[0]);     
//        
//        
//        //bytes = 512;
//        for (i = 0; i < bytes; i++)
//        {
//                //mspi_transceive(*buffer++);
//                  spiSendByte(*buffer++);  
//        }
//       
////        mspi_chip_release(AT45DB_CS);
//        delay_us(10);
//        CS_H();  
//        uart0_printf("END::");uart0_printDEC(bytes);
//        return;
//}

