#ifndef __STM32F1xx_LL_FLASH_H
#define __STM32F1xx_LL_FLASH_H

#include "stm32f1xx.h"

#define FLASH_PAGE_SIZE 0x400U

typedef enum
{
  ProgaraType_DATA64,
  ProgaraType_DATA32,
  ProgaraType_DATA16
} ProgaramDataType;

typedef enum
{
  FLASH_Lock = 1U,
  Flash_Unlock = !FLASH_Lock
} FlashStatus;



__STATIC_INLINE void
LL_FLASH_Lock(FLASH_TypeDef *FLASHx)
{
  SET_BIT(FLASHx->CR, FLASH_CR_LOCK);
}

/* @brief  Set flash erase type.
 * @param  FLASH_TYPEERASE specifies the FLASH flags to clear.
 *          This parameter can be any combination of the following values:
 *            @arg @ref FLASH_TYPEERASE_PAGES         PAGES Erase
 *            @arg @ref FLASH_TYPEERASE_MASSERASE      FLASH Write protected error flag
 * @retval none*/

__STATIC_INLINE void LL_FLASH_SetTypeErase(FLASH_TypeDef *FLASHx, uint32_t FLASH_TYPEERASE)
{
  SET_BIT(FLASHx->CR, FLASH_TYPEERASE);
}
/* @brief  Set flash erase ADDR.
 *          This parameter can be any combination of the following values:
 *            @arg @ref EraseADDR         uint32_t value
 * @retval none*/

__STATIC_INLINE void LL_FLASH_SetEraseADDR(FLASH_TypeDef *FLASHx, uint32_t EraseADDR)
{
  WRITE_REG(FLASHx->AR, EraseADDR);
}
/* @brief  Set flash erase ADDR.
 *          This parameter can be any combination of the following values:
 *            @arg @ref EraseADDR         uint32_t value
 * @retval none*/

__STATIC_INLINE void LL_FLASH_StartErase(FLASH_TypeDef *FLASHx)
{
  SET_BIT(FLASHx->CR, FLASH_CR_STRT);
}

/* @brief  Clear the specified FLASH flag.
 * @param  __FLAG__ specifies the FLASH flags to clear.
 *          This parameter can be any combination of the following values:
 *            @arg @ref FLASH_FLAG_EOP         FLASH End of Operation flag
 *            @arg @ref FLASH_FLAG_WRPERR      FLASH Write protected error flag
 *            @arg @ref FLASH_FLAG_PGERR       FLASH Programming error flag
 * @retval none*/

__STATIC_INLINE void LL_FLASH_ClearFlag(FLASH_TypeDef *FLASHx, uint32_t STATE_FLAG)
{
  WRITE_REG(FLASHx->SR, STATE_FLAG);
}

/*get bit flash bsy*/
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_BSY(FLASH_TypeDef *FLASHx)
{
  return (READ_BIT(FLASHx->SR, FLASH_SR_BSY) == (FLASH_SR_BSY));
}
/*get end of operation bilt*/
__STATIC_INLINE uint32_t LL_FLASH_IsActiveFlag_EOP(FLASH_TypeDef *FLASHx)
{
  return (READ_BIT(FLASHx->SR, FLASH_SR_EOP) == (FLASH_SR_EOP));
}
/*clear end of operation bilt*/
__STATIC_INLINE void LL_FLASH_ClearFlag_EOP(FLASH_TypeDef *FLASHx)
{
  SET_BIT(FLASHx->SR, FLASH_SR_EOP); // EOP bit Set clear
}
/* @brief  Set flash erase type.
 * @param  FLASH_TYPEERASE specifies the FLASH flags to clear.
 *          This parameter can be any combination of the following values:
 *            @arg @ref FLASH_TYPEERASE_PAGES         PAGES Erase
 *            @arg @ref FLASH_TYPEERASE_MASSERASE      FLASH Write protected error flag
 * @retval none*/
__STATIC_INLINE void LL_FLASH_DisableErase(FLASH_TypeDef *FLASHx, uint32_t FLASH_TYPEERASE)
{
  CLEAR_BIT(FLASHx->CR, FLASH_TYPEERASE);
}

/*EnableProgram*/
__STATIC_INLINE void LL_FLASH_EnableProgram(FLASH_TypeDef *FLASHx)
{
  SET_BIT(FLASHx->CR, FLASH_CR_PG);
}
/*DisenableProgram*/
__STATIC_INLINE void LL_FLASH_DisableProgram(FLASH_TypeDef *FLASHx)
{
  CLEAR_BIT(FLASHx->CR, FLASH_CR_PG);
}
/*read flash's states of lock or unlock*/
__STATIC_INLINE FlashStatus LL_FLASH_LockState(FLASH_TypeDef *FLASHx)
{
  return (FlashStatus)(READ_BIT(FLASHx->CR, FLASH_CR_LOCK));
}
/*set key for flash*/
__STATIC_INLINE void LL_FLASH_SetKey(FLASH_TypeDef *FLASHx, uint32_t key)
{
  WRITE_REG(FLASH->KEYR, key);
}

ErrorStatus LL_Flash_Unlock(void);
ErrorStatus LL_Flash_PageErase(uint32_t page_addr);
ErrorStatus LL_FLASH_Program_TwoBtye(uint32_t flash_addr, uint16_t data);
ErrorStatus LL_Flash_MassErase(void);
#endif
