import re


def main():
    with open(r'../data/galaxy.txt', 'r') as f:
        body = f.read()

    old = body
    body = re.sub(r'ap ap cons ([\d-]+) nil', r'[\1]', body)

    while True:
        old = body
        body = re.sub(r'ap ap cons ([\d-]+) \[([\d,-]+)\]', r'[\1,\2]', body)
        if old == body:
            break

    print(body)

    with open(r'../data/galaxy.txt', 'w') as f:
        f.write(body)


if __name__ == "__main__":
    main()
