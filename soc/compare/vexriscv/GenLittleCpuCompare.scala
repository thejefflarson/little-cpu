package vexriscv.demo

import vexriscv.plugin._
import vexriscv.{plugin, VexRiscv, VexRiscvConfig}
import spinal.core._

/**
 * VexRiscv for little-cpu's soc/compare/ harness.
 *
 * The build this replaces was FormalSimple -- riscv-formal's own verification
 * configuration -- which has NO MulPlugin, NO CsrPlugin and every hazard bypass
 * disabled. Measured against littlecpu's RV32IMAC_Zicsr_Zifencei_Zkt with traps,
 * CSRs, a timer and executor-only forwarding, that is not a peer: it flatters
 * VexRiscv on period (no bypass network, no CSR file, no multiplier to place)
 * and flatters littlecpu on cycles (nothing forwards, so everything stalls).
 *
 * The principle here is EACH CORE IN THE CONFIGURATION ITS OWN AUTHORS SHIP FOR
 * PERFORMANCE, at a comparable ISA. So this is GenFullNoMmuNoCache -- VexRiscv's
 * own no-MMU no-cache performance config, all four bypasses on -- with three
 * changes this harness requires:
 *
 *   FormalPlugin          -- soc/compare/bench_vexriscv.v reads rvfi_* to count
 *                            Dhrystone's writes. Without it the bench will not
 *                            elaborate.
 *   compressedGen = true  -- C, which littlecpu has.
 *   no DebugPlugin        -- the bench wires no debug bus, and an unwired debug
 *                            port is still logic the placer must place.
 *
 * Deliberately NOT hobbled to match littlecpu's narrower forwarding. Picking a
 * weaker config for the other core earns the same criticism in reverse.
 *
 * resetVector is 0 because the harness's ROM is at 0, not 0x80000000.
 */
object GenLittleCpuCompare extends App {
  def cpu() = new VexRiscv(
    config = VexRiscvConfig(
      plugins = List(
        new FormalPlugin,
        new IBusSimplePlugin(
          resetVector = 0x00000000l,
          cmdForkOnSecondStage = false,
          cmdForkPersistence = false,
          prediction = STATIC,
          catchAccessFault = false,
          compressedGen = true
        ),
        new DBusSimplePlugin(
          catchAddressMisaligned = false,
          catchAccessFault = false
        ),
        new DecoderSimplePlugin(
          catchIllegalInstruction = true
        ),
        new RegFilePlugin(
          regFileReadyKind = plugin.SYNC,
          zeroBoot = false
        ),
        new IntAluPlugin,
        new SrcPlugin(
          separatedAddSub = false,
          executeInsertion = true
        ),
        new FullBarrelShifterPlugin,
        new HazardSimplePlugin(
          bypassExecute           = true,
          bypassMemory            = true,
          bypassWriteBack         = true,
          bypassWriteBackBuffer   = true,
          pessimisticUseSrc       = false,
          pessimisticWriteRegFile = false,
          pessimisticAddressMatch = false
        ),
        new MulPlugin,
        new DivPlugin,
        new CsrPlugin(CsrPluginConfig.small),
        new BranchPlugin(
          earlyBranch = false,
          catchAddressMisaligned = true
        ),
        new YamlPlugin("cpu0.yaml")
      )
    )
  )

  SpinalConfig(
    defaultConfigForClockDomains = ClockDomainConfig(
      resetKind = spinal.core.SYNC,
      resetActiveLevel = spinal.core.HIGH
    )
  ).generateVerilog(cpu())
}
