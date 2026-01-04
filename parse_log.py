import sys
import re

def extract_strings(filename):
    with open(filename, 'rb') as f:
        # Read first 5MB to avoid memory issues if huge
        content = f.read(5 * 1024 * 1024) 
        # Find readable strings > 4 chars
        matches = re.findall(b'[a-zA-Z0-9_ ./:()-]{4,}', content)
        for m in matches:
            try:
                s = m.decode('utf-8')
                if "Error" in s or "Exception" in s or "Command" in s or "Vision" in s or "CAN" in s:
                    print(s)
            except:
                pass

if __name__ == "__main__":
    if len(sys.argv) > 1:
        extract_strings(sys.argv[1])
