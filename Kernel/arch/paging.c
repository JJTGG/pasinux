#include "paging.h"
#include "mm_fs.h"
#include "serial.h"

#include <stddef.h>
#include <stdint.h>

#ifdef FREESTANDING

void paging_init_higher_half(void)
{
    uint32_t pde_val = page_directory[PAGING_PDE_HIGHER];

    if (pde_val & PAGING_FLAG_PRESENT) {
        serial_puts("[PAGING] higher-half active: kernel at 0xC0000000+\n");
    } else {
        serial_puts("[PAGING] WARNING: PDE[768] not present!\n");
    }
}

uint32_t *paging_create_pd(void)
{
    /*
     * Allocate enough space for alignment because kmalloc() does not
     * guarantee a 4 KiB-aligned result.
     *
     * The returned pointer is a kernel virtual address. The corresponding
     * physical address is obtained through paging_phys_addr() when the PD
     * is installed into CR3.
     */
    void *raw = kmalloc(PAGING_PAGE_SIZE * 2u);
    if (!raw) {
        serial_puts("[PAGING] kmalloc failed for process PD\n");
        return NULL;
    }

    uintptr_t raw_addr = (uintptr_t)raw;
    uintptr_t aligned =
        (raw_addr + PAGING_PAGE_SIZE - 1u) &
        ~(uintptr_t)(PAGING_PAGE_SIZE - 1u);

    uint32_t *pd = (uint32_t *)aligned;

    for (uint32_t i = 0u; i < PAGING_PDE_COUNT; ++i) {
        pd[i] = 0u;
    }

    /*
     * Copy only the kernel's higher-half mappings.
     *
     * PDE[768] is supervisor-only in the bootstrap page directory, so
     * user processes inherit the kernel mapping but cannot access it from
     * ring 3.
     */
    for (uint32_t i = PAGING_PDE_HIGHER;
         i < PAGING_PDE_COUNT;
         ++i) {
        pd[i] = page_directory[i] & ~PAGING_FLAG_USER;
    }

    return pd;
}

int paging_map_page(uint32_t *pd, uint32_t vaddr, uint32_t phys,
                    uint32_t flags)
{
    if (!pd) {
        return -1;
    }

    /*
     * A normal 4 KiB page mapping requires both addresses to be page
     * aligned. Silently masking them would map a different address than
     * the caller requested.
     */
    if ((vaddr & (PAGING_PAGE_SIZE - 1u)) != 0u ||
        (phys  & (PAGING_PAGE_SIZE - 1u)) != 0u) {
        return -1;
    }

    /*
     * Do not allow unknown paging bits to be passed into a PTE.
     *
     * At the moment this kernel supports PRESENT, WRITABLE and USER.
     * PS belongs to PDEs and must never be placed into a normal PTE.
     */
    if (flags & ~(PAGING_FLAG_PRESENT |
                  PAGING_FLAG_WRITABLE |
                  PAGING_FLAG_USER)) {
        return -1;
    }

    /*
     * PRESENT is controlled by this function, so callers do not need
     * to provide it and cannot accidentally create a non-present mapping.
     */
    flags &= ~PAGING_FLAG_PRESENT;

    uint32_t pde_idx = vaddr >> 22u;
    uint32_t pte_idx = (vaddr >> 12u) & 0x3FFu;

    uint32_t pde = pd[pde_idx];

    /*
     * A PDE with PS set is a 4 MiB mapping, not a pointer to a page table.
     *
     * Treating it as a page-table PDE would take bits from the 4 MiB
     * mapping as a physical page-table address and write through that
     * bogus address.
     *
     * This is especially important for PDE[768], which is the kernel's
     * bootstrap 4 MiB higher-half mapping.
     */
    if (pde & PAGING_FLAG_PS) {
        return -1;
    }

    if (!(pde & PAGING_FLAG_PRESENT)) {
        /*
         * Allocate enough space to obtain a 4 KiB-aligned page table.
         * The allocator currently does not expose a dedicated physical
         * page allocator, so retain the existing kmalloc-based approach.
         */
        void *raw = kmalloc(PAGING_PAGE_SIZE * 2u);
        if (!raw) {
            return -1;
        }

        uintptr_t raw_addr = (uintptr_t)raw;
        uintptr_t aligned =
            (raw_addr + PAGING_PAGE_SIZE - 1u) &
            ~(uintptr_t)(PAGING_PAGE_SIZE - 1u);

        uint32_t *pt = (uint32_t *)aligned;

        for (uint32_t i = 0u; i < PAGING_PTE_COUNT; ++i) {
            pt[i] = 0u;
        }

        uint32_t pt_phys = paging_phys_addr(pt);

        /*
         * The page-table USER bit must be present if the mapped page is
         * intended to be accessible from ring 3. Kernel-only mappings
         * remain supervisor-only.
         */
        pd[pde_idx] =
            pt_phys |
            PAGING_FLAG_PRESENT |
            PAGING_FLAG_WRITABLE |
            (flags & PAGING_FLAG_USER);

        pde = pd[pde_idx];
    }

    /*
     * Re-check PS after creating/fetching the PDE. This keeps the
     * invariant explicit and protects this path if the implementation
     * changes later.
     */
    if (pde & PAGING_FLAG_PS) {
        return -1;
    }

    uint32_t pt_phys = pde & 0xFFFFF000u;

    if (pt_phys == 0u) {
        return -1;
    }

    uint32_t *pt = (uint32_t *)paging_virt_addr(pt_phys);

    /*
     * Preserve the caller's requested page permissions. The PDE itself
     * already carries the USER permission needed to reach this PTE.
     */
    pt[pte_idx] =
        (phys & 0xFFFFF000u) |
        flags |
        PAGING_FLAG_PRESENT;

    /*
     * Flush the mapping from the current CPU's TLB.
     */
    __asm__ volatile ("invlpg (%0)" : : "r"(vaddr) : "memory");

    return 0;
}

#endif