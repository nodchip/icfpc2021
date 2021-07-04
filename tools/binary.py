import collections

def main():
    # numbers = collections.OrderedDict()
    # with open('numbers.txt', 'r') as f:
    #     for line in f:
    #         number = int(line)
    #         if number not in numbers:
    #             numbers[number] = 0
    #         numbers[number] = numbers[number] + 1
    # numbers = list(numbers.items())
    # numbers.sort(key=lambda x:x[1], reverse=True)
    # print(len(numbers))
    # print(numbers)

    with open('numbers.txt', 'r') as f:
        for line in f:
            number = int(line) + 2**32
            print(bin(number))


if __name__ == "__main__":
    main()
