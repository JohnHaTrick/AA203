# AA203 Drift Initiation

## Getting started:
- run SimAndOptHarness.m
  - calls: DiftNonlinear.m
    - calls: mdlvar.m (what's this do?)
- Dynamics equations located in: DroftDynamics.pdf
  - see neat model sketch in: ModelSketch.docx
- Parameters located in: DriftParams.xlsx
  - TODO: move this param info to .m script
  - Calculate drift equilibrium params in /DriftEquilibriumScripts/ExtractFinalParams.m
- Dependancies:
  - matlab (>2016?)
  - YALMIP: https://yalmip.github.io/download/
  - IPOPT: https://www.inverseproblem.co.nz/OPTI/index.php/DL/DownloadOPTI
  
## Questions:
- What are:
  - matlabfrag.m?
  - mlf2pdf.m?
