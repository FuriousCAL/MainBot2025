import sys
import re
import os

def extract_errors(filename):
    with open(filename, 'rb') as f:
        content = f.read()
        matches = re.findall(b'[a-zA-Z0-9_ ./:()-]{4,}', content)
        
        for m in matches:
            try:
                s = m.decode('utf-8')
                if "Exception" in s or "Error" in s or "Watchdog" in s:
                    print(s)
            except:
                pass

if __name__ == "__main__":
    if len(sys.argv) > 1:
        extract_errors(sys.argv[1])
