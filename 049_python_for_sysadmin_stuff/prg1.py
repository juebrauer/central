import sys

if len(sys.argv) > 1:
    for arg in sys.argv[1:]:
        print(f"Argument: {arg}")
else:
    print("Keine Argumente übergeben.")