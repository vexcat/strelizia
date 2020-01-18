_HEAP_SIZE = DEFINED(_HEAP_SIZE) ? _HEAP_SIZE : 0x02E00000;  /* ~48 MB */
  
start_of_cold_mem = 0x03800000;
_COLD_MEM_SIZE = 0x00400000;
end_of_cold_mem = start_of_cold_mem + _COLD_MEM_SIZE;

start_of_hot_mem = 0x06A00000;
_HOT_MEM_SIZE = 0x01600000;
end_of_hot_mem = start_of_hot_mem + _HOT_MEM_SIZE;

MEMORY
{
  /* user code  72M */
  COLD_MEMORY : ORIGIN = start_of_cold_mem, LENGTH = _COLD_MEM_SIZE /* 4MiB */
  HEAP : ORIGIN = 0x03C00000, LENGTH = _HEAP_SIZE
  HOT_MEMORY : ORIGIN = 0x06a00000, LENGTH = 0x00200000  /* Only 2MiB here, meant only for initialization data. */
  auto_MEMORY : ORIGIN = 0x6c00000, LENGTH = 0x13cd4e9
default_MEMORY : ORIGIN = 0x7fcd4e9, LENGTH = 0x32b17
}
