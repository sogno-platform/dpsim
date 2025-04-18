{
  fetchFromGitHub,
  sundials,
  python2,
}:
(sundials.overrideAttrs (
  finalAttrs: prevAttrs: {
    version = "3.2.1";
    src = fetchFromGitHub {
      owner = "LLNL";
      repo = "sundials";
      rev = "v${finalAttrs.version}";
      hash = "sha256-5fVgxFEzhzw7rAENpt2+8qGR0pe00nntSFnyArmafzU";
    };

    doCheck = false;
  }
)).override
  { python = python2; }
