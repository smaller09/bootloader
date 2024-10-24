#include "stm32f1xx_ll_flash.h"
#include "stm32f1xx_ll_utils.h"

void LL_FLASH_Wait_for_Busy()
{
        while (LL_FLASH_IsActiveFlag_BSY(FLASH))
        {
                __NOP();
        }
}
ErrorStatus LL_FLASH_Program_TwoBtye(__IO uint16_t *flash_addr, uint16_t *data)
{
        LL_FLASH_Wait_for_Busy();

        LL_FLASH_EnableProgram(FLASH);

        *flash_addr = *data;

        LL_FLASH_Wait_for_Busy();

        if (LL_FLASH_IsActiveFlag_EOP(FLASH))
        {
                LL_FLASH_ClearFlag_EOP(FLASH);
        }
        else
        {
                return ERROR;
        }
        LL_FLASH_DisableProgram(FLASH);
        return SUCCESS;
}

ErrorStatus LL_Flash_Unlock(void)
{
   //     LL_FLASH_Wait_for_Busy();
        if (LL_FLASH_LockState(FLASH))
        {
                LL_FLASH_SetKey(FLASH, FLASH_KEY1);
                LL_FLASH_SetKey(FLASH, FLASH_KEY2);
        }
        return SUCCESS;
}

ErrorStatus LL_Flash_PageErase(uint32_t page_addr)
{

        LL_FLASH_SetTypeErase(FLASH, FLASH_CR_PER);
        LL_FLASH_SetEraseADDR(FLASH, page_addr);
        LL_FLASH_StartErase(FLASH);
        LL_FLASH_Wait_for_Busy();
        if (LL_FLASH_IsActiveFlag_EOP(FLASH))
        {
                LL_FLASH_ClearFlag_EOP(FLASH);
        }
        else
        {
                return ERROR;
        }
        LL_FLASH_DisableErase(FLASH, FLASH_CR_PER);
        return SUCCESS;
}

ErrorStatus LL_Flash_MassErase(void)
{

        LL_FLASH_SetTypeErase(FLASH, FLASH_CR_MER);
        LL_FLASH_StartErase(FLASH);
        LL_FLASH_Wait_for_Busy();

        if (LL_FLASH_IsActiveFlag_EOP(FLASH))
        {
                LL_FLASH_ClearFlag_EOP(FLASH);
        }
        else
        {
                return ERROR;
        }
        LL_FLASH_DisableErase(FLASH, FLASH_CR_MER);
        return SUCCESS;
}