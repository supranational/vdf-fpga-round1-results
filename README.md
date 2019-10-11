# VDF Alliance FPGA Contest Round 1 Results

**Congratulations to Eric Pearson for his winning entry with 28.6 ns/sq latency!!**

Thank you to all those who participated and submitted entries!

This repository contains the results and designs submitted for round 1 of the FPGA competition. See [FPGA Contest Wiki](https://supranational.atlassian.net/wiki/spaces/VA/pages/36569208/FPGA+Contest) for more information about the contest.

Five teams submitted a total of 10 designs in round 1. All were based on the provided Ozturk baseline and optimized in various ways for the FPGA platform. 

**The round 2 baseline latency will be set at 28.6 ns/sq based on the winning entry from Eric.**

For updates on Round 2 of the competition please see [FPGA Contest Round 2](https://supranational.atlassian.net/wiki/spaces/VA/pages/48758785/FPGA+Competition+Round+2)

## Results

The following designs were submitted and are available in this repository. The submission_form.txt file provides information from the contestants about their design. 

Designs were evaluated for performance as follows:
  * Run hardware emulation to test basic functionality
  * Synthesis using the provided scripts
  * Run for 2^30 iterations on AWS F1 and check for correctness and performance.

Unfortunately we had a number of designs that where not fully functional for varous reasons. Please see the round 2 wiki page linked above for further discussion on what happened and advice for round 2.


DNQ = Did not qualify

Team Name | Directory | Expected | Actual
----------|-----------|----------|-------
Eric Pearson | eric_pearson-1 | 31.5 ns/sq | 31.5 ns/sq
Eric Pearson | eric_pearson-2 | 28.6 ns/sq | 28.6 ns/sq
FPGA Enthusiast | fpga_enthusiast-1 | 45.7 ns/sq | Clocking issues caused slowdown to 56 ns/sq
FPGA Enthusiast | fpga_enthusiast-2 | 45.7 ns/sq | DNQ - did not route successfully
FPGA Enthusiast | fpga_enthusiast-3 | 45.7 ns/sq | DNQ - did not meet timing
Silicon Tailor | silicon_tailor-30 | 30 ns/sq | DNQ - clocking/power faults on F1
Silicon Tailor | silicon_tailor-291 | 29.1 ns/sq | DNQ - clocking/power faults on F1
Silicon Tailor | silicon_tailor-296 | 29.6 ns/sq | DNQ - clocking/power faults on F1
Geriatric Guys with Gates | geriatric_guys_with_gates | 44.96 ns/sq | 48 ns/sq - slower than expected due to clocking issues
Andreas Brokalakis | andreas_brokalakis | 24 ns/sq | DNQ - simulation/synthesis errors

## Optimization ideas for round 2

- Combine concepts from the round 1 submissions
- Improve upon round 1 submissions
- Explore approaches and algorithms other than Ozturk, such as Chinese Remainder Theorem
- Explore options to do fewer reductions, such as squaring twice then reducing
