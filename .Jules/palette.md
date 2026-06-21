## 2024-05-24 - CLI Usage Formatting
**Learning:** In bash scripts like `cli/psy`, define ANSI color variables using bash ANSI-C quoting and conditionally apply them using `if [ -t 1 ]; then` to prevent garbage characters from appearing when the output is piped.
**Action:** Apply this pattern when designing bash CLI help messages, and group commands logically for improved CLI UX.
