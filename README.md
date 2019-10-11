# VDF Alliance FPGA Contest Round 1 Results

This repository contains the results and designs submitted for round 1 of the FPGA competition. See [FPGA Contest Wiki](https://supranational.atlassian.net/wiki/spaces/VA/pages/36569208/FPGA+Contest) for more information. 

Overall we had good turnout, with 5 teams submitting a total of 10 design variations. All were based on the provided Ozturk baseline and optimized in various ways for the FPGA platform. 

**The round 2 baseline latency will be set at 28.6 ns/sq based on the winning entry from Eric Pearson.** For round 2 would still very much like to see other approaches and algorithms come into play as well as the ideas represented here combined and improved upon further. 

Congratulations to Eric for his winning entry, a big thanks to all those who participated and submitted entries, and good luck in round 2!

For updates on Round 2 of the competition please see https://supranational.atlassian.net/wiki/spaces/VA/pages/36569208/FPGA+Competition+Round+2
  
## Submissions 

The following designs were submitted and are available in this repository.

Team Name | Directory | Notes
----------|-----------|------
Eric Pearson | eric_pearson-1 | Targets latency of 31.5ns/sq
Eric Pearson | eric_pearson-2 | Reduces latency target to 28.6 ns/sq
FPGA Enthusiast | fpga_enthusiast-1 | Targets 45.7ns/sq, provided feedback on clocking issues
FPGA Enthusiast | fpga_enthusiast-2 | Incorporates MMCM (PLL), provided feedback fails routing
FPGA Enthusiast | fpga_enthusiast-3 | Resolves routing issue
Silicon Tailor | silicon_tailor-30 | Targets 30ns/sq
Silicon Tailor | silicon_tailor-291 | Targets 29.1ns/sq
Silicon Tailor | silicon_tailor-296 | Targets 29.6ns/sq
Geriatric Guys with Gates | geriatric_guys_with_gates | Single submission from team targeting 44.96ns/sq
Andreas Brokalakis | andreas_brokalakis | Single submission from team targeting 24ns/sq

## Results

Designs were evaluated for performance as follows:
  * Run hardware emulation to test basic functionality
  * Synthesis using the provided scripts
  * Run for 2^30 iterations on AWS F1 and check for correctness and performance.

DNQ = Did not qualify

Team Name | Directory | Notes
----------|-----------|------
Eric Pearson | eric_pearson-1 | 31.5 ns/sq
Eric Pearson | eric_pearson-2 | 28.6 ns/sq
FPGA Enthusiast | fpga_enthusiast-1 | Clocking issues caused slowdown to 56 ns/sq
FPGA Enthusiast | fpga_enthusiast-2 | DNQ - routing issues
FPGA Enthusiast | fpga_enthusiast-3 | DNQ - did not meet timing
Silicon Tailor | silicon_tailor-30 | DNQ - clocking/power faults on F1
Silicon Tailor | silicon_tailor-291 | DNQ - clocking/power faults on F1
Silicon Tailor | silicon_tailor-296 | DNQ - clocking/power faults on F1
Geriatric Guys with Gates | geriatric_guys_with_gates | 48 ns/sq
Andreas Brokalakis | andreas_brokalakis | DNQ - simulation/synthesis errors

