import argparse
import modulate
import http.client

if __name__ == '__main__':
    parser = argparse.ArgumentParser('modulate')
    parser.add_argument('--api_key', type=str, required=True)
    parser.add_argument('value', type=str)
    args = parser.parse_args()

    api_key = args.api_key
    value = args.value

    print('value:')
    print(value)

    body = eval(value)
    print('body:')
    print(body)

    body = modulate.modulate(body)
    body = str(body)
    body = body.encode()

    headers = {
        "Content-type": "application/x-www-form-urlencoded",
        "Accept": "text/plain"
        }
    conn = http.client.HTTPSConnection('icfpc2020-api.testkontur.ru')
    conn.request("POST", f"/aliens/send?apiKey={api_key}", body, headers)
    response = conn.getresponse()
    print(f'response.status={response.status}')
    print(f'response.reason={response.reason}')
    return_value = response.read().decode()
    print(return_value)
    # print(modulate.demodulate(return_value))
    print(modulate.demodulate_readable(return_value))
