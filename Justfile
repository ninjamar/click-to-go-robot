set default-list := true

format:
    #!/usr/bin/env zsh

    ruff format . 
    isort .
    clang-format -i $(fd -e cpp -e hpp) # fd respects gitignore

lint:
    #!/usr/bin/env zsh

    ruff check .
    ty check .
    clang-tidy $(fd -e cpp -e hpp)