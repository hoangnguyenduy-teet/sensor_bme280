#include "error_codes.h"
#include "atomic.h"
#include "spi_io.h"
#include "stddef.h"

/**
 * @brief Initialize the SPI peripheral with the provided configuration
 * @param me Pointer to the SPI_Io_t structure
 * @param init Pointer to the SPI_IoInit_t structure containing initialization parameters
 * @return ERROR_OK on success, or an error code on failure
 */
uint32_t spi_io_init(SPI_Io_t *me, const SPI_IoInit_t *init)
{
    if (me == NULL || init == NULL || init->hSpi == NULL) {
        return ERROR_INVALID_PARAM;
    }

    // Copy initialization parameters to the SPI_Io_t structure
    me->hSpi = init->hSpi;
    me->ui32MaxSpeedHz = init->ui32MaxSpeedHz;
    me->mode = init->mode;
    me->bitOrder = init->bitOrder;
    me->lanes = init->lanes;



    return ERROR_OK;
}
uint32_t spi_io_transfer_sync(SPI_Io_t *me, uint8_t	*pui8TxBuff, uint8_t	*pui8RxBuff, uint32_t ui32Length)
{

	SPI_TypeDef * SPIx = me->hSpi;

	for (uint32_t ui32Idx = 0; ui32Idx < ui32Length; ui32Idx++)
	{
		while (!(SPIx->SR & SPI_SR_TXE));
		if (pui8TxBuff) 	SPIx->DR = pui8TxBuff[ui32Idx];
		else 				SPIx->DR = 0xAA;

		while (!(SPIx->SR & SPI_SR_RXNE)) ;
		if (pui8RxBuff) 		pui8RxBuff[ui32Idx] = SPIx->DR;
		else 	(void)SPIx->DR;
	}

	return ERROR_OK;
}
uint32_t spi_io_write_sync(SPI_Io_t *me, uint8_t	*pui8TxBuff, uint32_t ui32Length)
{
	return spi_io_transfer_sync(me,pui8TxBuff,NULL,ui32Length);
}
uint32_t spi_io_read_sync(SPI_Io_t *me, uint8_t	*pui8RxBuff, uint32_t ui32Length)
{
	return spi_io_transfer_sync(me,NULL, pui8RxBuff,ui32Length);
}
uint32_t spi_io_write_and_read_sync(SPI_Io_t *me, uint8_t	*pui8TxBuff, uint32_t ui32TxLength, uint8_t	*pui8RxBuff,uint32_t ui32RxLength)
{
	if ((!pui8TxBuff) || (!pui8RxBuff))	return ERROR_INVALID_PARAM;

	SPI_TypeDef * SPIx = me->hSpi;
	for (uint32_t ui32Idx = 0; ui32Idx < ui32TxLength; ui32Idx++)
	{
		while (!(SPIx->SR & SPI_SR_TXE)) ;
		if (pui8TxBuff) 	SPIx->DR = pui8TxBuff[ui32Idx];
		while (!(SPIx->SR & SPI_SR_RXNE)) ;
		(void)SPIx->DR;
	}
	for (uint32_t ui32Idx = 0; ui32Idx < ui32RxLength; ui32Idx++)
	{
		while (!(SPIx->SR & SPI_SR_TXE));
		SPIx->DR = 0xAA;
		while (!(SPIx->SR & SPI_SR_RXNE)) ;
		pui8RxBuff[ui32Idx] = SPIx->DR;
	}

	return ERROR_OK;
}
