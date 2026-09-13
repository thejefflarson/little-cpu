// start.S must set gp before any linker-relaxed gp-relative reference runs; left unset, gp
// is the register file's reset zero and a gp-relative store lands near address 0.
volatile int tohost __attribute__((section(".tohost"))) = 0;

extern char __global_pointer$[];

int main(void) {
  void *gp_value;
  __asm__ __volatile__("mv %0, gp" : "=r"(gp_value));
  tohost = (gp_value == (void *)__global_pointer$) ? 1 : 3;
  return 0;
}
