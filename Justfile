
format:
    #!/usr/bin/env zsh

    ruff format . 
    isort .
    clang-format -i ./**/*.{cpp,hpp}