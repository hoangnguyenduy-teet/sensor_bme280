#include "usart_io.h"
#include "error_codes.h"
#include "stm32f1xx_hal_usart.h"
#include "stdio.h"
#include "atomic.h"

// Kích hoạt UART
void uart_io_active(UsartIo_t * me) {
    if ((me->hUsart != NULL) && (!me->active)) {
        // Bật UART
        HAL_USART_Enable(me->hUsart);
        // Bật ngắt nhận (RXNE)
        HAL_USART_EnableIT_RXNE(me->hUsart);
        me->active = true;
    }
}

void uart_io_deactive(UsartIo_t * me) {
    if (me->hUsart != NULL && me->active) {
        // Tắt ngắt TXE và RXNE
        HAL_USART_DisableIT_TXE(me->hUsart);
        HAL_USART_DisableIT_RXNE(me->hUsart);
        // Tắt UART
        HAL_USART_Disable(me->hUsart);
        me->tx_busy = false; // Đặt lại trạng thái truyền
        me->active = false;
    }
}

// Kiểm tra trạng thái hoạt động
bool uart_io_is_active(UsartIo_t* me) {
    return me->active;
}

// Đọc dữ liệu từ bộ đệm nhận
uint32_t uart_io_read(UsartIo_t* me, uint8_t* Buffer, uint32_t ui32MaxLen) {

    uint32_t ui32Count = 0;
    uint32_t ui32Result;

    for (ui32Count = 0;ui32Count < ui32MaxLen;ui32Count++) {
    	ui32Result = ring_buf_get(&me->rxRingBuf, Buffer);
        if (ui32Result) {
            break;
        }
        Buffer++;
    }
    return ui32Count ;
}

// Đọc dữ liệu từ bộ đệm nhận
uint32_t uart_io_read_byte(UsartIo_t* me, uint8_t* c) {

    uint32_t ui32Result;

    	ui32Result = ring_buf_get(&me->rxRingBuf, c);

    return ui32Result ;
}

// Ghi dữ liệu vào bộ đệm phát
uint32_t uart_io_write(UsartIo_t* me, const uint8_t * pui8Buffer, uint32_t ui32NumData) {

    uint32_t ui32Count = 0;

    // Nếu không có truyền đang diễn ra, kích hoạt truyền
	ENTER_CRITICAL();
    if (!me->tx_busy) {

        HAL_USART_TransmitData8(me->hUsart, *pui8Buffer);
        me->tx_busy = true;
        HAL_USART_EnableIT_TXE(me->hUsart); // Bật ngắt TXE

        ui32Count ++;
    }
    EXIT_CRITICAL();
    // Thêm dữ liệu  vào bộ đệm
    while (ui32Count < ui32NumData ) {
        if (ring_buf_put(&me->txRingBuf, pui8Buffer[ui32Count]) == ERROR_OK)     ui32Count++;
        else break;
    }

    return ui32Count;
}
// Ghi dữ liệu vào bộ đệm phát
uint32_t uart_io_write_char(UsartIo_t* me, const uint8_t ui8Val) {
    // Nếu không có truyền đang diễn ra, kích hoạt truyền
	uint32_t ui32RetVal = ERROR_BUFFER_FULL;
	ENTER_CRITICAL();
    if (!me->tx_busy) {

        HAL_USART_TransmitData8(me->hUsart, ui8Val);
        me->tx_busy = true;
        HAL_USART_EnableIT_TXE(me->hUsart); // Bật ngắt TXE

        ui32RetVal = ERROR_OK;
    }
    else
    {
    // Thêm dữ liệu  vào bộ đệm
        if (ring_buf_put(&me->txRingBuf, ui8Val) == ERROR_OK)     ui32RetVal = ERROR_OK;;

    }
    EXIT_CRITICAL();
    return ui32RetVal;
}

// Callback xử lý ngắt nhận
void uart_io_rx_callback(UsartIo_t* me) {
    if (HAL_USART_IsActiveFlag_RXNE(me->hUsart)) {
        uint8_t received_data = HAL_USART_ReceiveData8(me->hUsart);
        ring_buf_put(&me->rxRingBuf, received_data);
    }

    if (HAL_USART_IsActiveFlag_ORE(me->hUsart))
    	HAL_USART_ClearFlag_ORE(me->hUsart);
}

// Callback xử lý ngắt phát
void uart_io_tx_callback(UsartIo_t* me) {
    uint8_t data;
    uint32_t result = ERROR_OK;

    if (HAL_USART_IsActiveFlag_TXE(me->hUsart)) {

        result = ring_buf_get(&me->txRingBuf, &data);
        if (!result) {
            // Tiếp tục truyền byte tiếp theo
            HAL_USART_TransmitData8(me->hUsart, data);
        } else {
            // Không còn dữ liệu, tắt ngắt TXE và đặt trạng thái không bận
            HAL_USART_DisableIT_TXE(me->hUsart);
            me->tx_busy = false;
        }
    }
}
