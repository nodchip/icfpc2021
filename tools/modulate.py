import sys

def list_to_tuple(value):
    if not isinstance(value, list):
        return value
    if len(value) == 0:
        return None
    return (value[0], list_to_tuple(value[1:]))

def tuple_to_list(value):
    if value is None:
        return []
    if not isinstance(value, tuple):
        return value
    assert len(value) == 2
    second = tuple_to_list(value[1])
    if isinstance(second, list):
        return [tuple_to_list(value[0]), *second]
    else:
        return (tuple_to_list(value[0]), second)

def modulate(value):
    value = list_to_tuple(value)
    if isinstance(value, int):
        bits = ['01' if value >= 0 else '10']
        magnitude = abs(value)
        num_bits = 0
        while magnitude >= 1 << num_bits:
            num_bits += 4
            bits.append('1')
        bits.append('0')
        for i in reversed(range(num_bits)):
            bits.append(str((magnitude >> i) & 1))
        return ''.join(bits)
    if value is None:
        return '00'
    assert len(value) == 2
    bits = ['11']
    for element in value:
        bits.append(modulate(element))
    return ''.join(bits)

def demodulate(bits, start=0):
    if bits.startswith(('01', '10'), start):
        sign = 1 if bits.startswith('01', start) else -1
        start += 2
        num_bits = (bits.find('0', start) - start) * 4
        start += num_bits // 4 + 1
        value = 0 if num_bits == 0 else sign * int(bits[start:start+num_bits], 2)
        return value, start + num_bits
    if bits.startswith('00', start):
        return None, start + 2
    assert bits.startswith('11', start)
    start += 2
    value_list = []
    for i in range(2):
        element, start = demodulate(bits, start)
        value_list.append(element)
    return tuple(value_list), start

def demodulate_readable(bits):
    value, start = demodulate(bits)
    assert start == len(bits)
    return tuple_to_list(value)

def test():
    for value in (0, 1, -1, 2, -2, 16, -16, 255, -255, 256, -256,
                  [], [[]], [0], (1, 2), [1, 2], [1, [2, 3], 4]):
        binary = modulate(value)
        print('{}: {}'.format(value, binary))
        assert demodulate_readable(binary) == value

if __name__ == '__main__':
    if len(sys.argv) == 2:
        print(demodulate_readable(sys.argv[1]))
    else:
        test()
