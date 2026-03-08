## 2024-05-18 - CLI Help Menu Semantic Grouping and TTY Colors
**Learning:** Adding colors to CLI tools makes it more pleasant and readable, but doing so via piping can generate garbage characters. Also, a long list of commands can be overwhelming.
**Action:** When designing CLI help menus using shell scripts, define ANSI colors utilizing bash ANSI-C quoting and conditionally apply them using `[ -t 1 ]`. Group commands into logical, semantic sections.
