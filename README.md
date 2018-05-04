# AA203 Drift Initiation

## Getting started:
- run Optimization2018.m
  - calls: DiftNonlinear.m
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
  - matlabfrag.m: Generates figures
  - mdlvar.m: sdpvar with normalization
  - mlf2pdf.m: Generates pdf figures using matlabfrag
