"""Offline verification for the one-shot maintenance boot latch."""
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory
from elftools.elf.elffile import ELFFile

def maintenance_address(elf_path, image):
    with Path(elf_path).open('rb') as stream:
        elf=ELFFile(stream)
        symbols=elf.get_section_by_name('.symtab').get_symbol_by_name('local_home_boot_inhibit')
        if not symbols or len(symbols)!=1 or symbols[0]['st_size']!=8:
            raise RuntimeError('Maintenance symbol missing/invalid; no flash performed')
        symbol=symbols[0];section=elf.get_section(symbol['st_shndx'])
        if section['sh_type']!='SHT_NOBITS' or 'noinit' not in section.name:
            raise RuntimeError('Maintenance latch is not in noinit')
        address=symbol['st_value']
        if not (0x20000000<=address<0x20050000 or 0x30000000<=address<0x30050000):
            raise RuntimeError('Maintenance address not MCXN947 SRAM')
    # Same conversion/gap fill as Zephyr. ELF load segments include linker gaps
    # with different bytes; comparing segments directly would reject good images.
    with TemporaryDirectory(prefix='panthera-image-') as directory:
        output=Path(directory)/'image.bin'
        subprocess.run(['/home/aster/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-objcopy',
                        '--gap-fill','0xFF','--output-target=binary',
                        '--remove-section=.comment','--remove-section=COMMON',
                        str(elf_path),str(output)],check=True)
        if output.read_bytes()!=image:
            raise RuntimeError('ELF/image mismatch; no flash performed')
    return address
