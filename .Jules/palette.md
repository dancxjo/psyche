## 2024-03-11 - Dynamic ANSI Colors in Bash Help Messages
**Learning:** Using conditional `[ -t 1 ]` with unquoted heredocs allows dynamically injecting ANSI escape sequences for TTY outputs while cleanly stripping them for piped commands. Using bash ANSI-C quoting (`$'\e[34m'`) ensures reliability across shells.
**Action:** When creating CLI tools, check if standard output is a TTY (`[ -t 1 ]`) to optionally enable color variables injected into heredocs to prevent garbled outputs when users pipe `help` messages to `grep` or other tools.
