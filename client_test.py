import socket

def main(host="127.0.0.1", port=9000):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    print("[client] connected, receiving lines...")
    f = s.makefile("r", encoding="utf-8", newline="\n")
    for i, line in enumerate(f):
        print(line.strip())
        if i > 20:
            break
    s.close()

if __name__ == "__main__":
    main()