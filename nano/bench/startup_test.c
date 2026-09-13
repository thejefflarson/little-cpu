// Regression for nano/bench/start.S's own gp initialization. GCC's default linker
// relaxation turns a `la` of a symbol within +-2KB of __global_pointer$ into a single
// gp-relative addi, so every startup here must set gp before any such reference runs.
// Left unset, gp reads this simulator's register-file reset value of zero, and a
// gp-relative store lands near address 0 instead of wherever the symbol actually is.
volatile int tohost __attribute__((section(".tohost"))) = 0;

extern char __global_pointer$[];

int main(void) {
  void *gp_value;
  __asm__ __volatile__("mv %0, gp" : "=r"(gp_value));
  tohost = (gp_value == (void *)__global_pointer$) ? 1 : 3;
  return 0;
}
