#!/usr/bin/python3

import optparse
import requests

API_TOKEN = 'f086e0bb-cfeb-495d-9f1d-240ea717b30b'

def get_options():
    parser = optparse.OptionParser()
    parser.add_option('--hello', action='store_true', help='Say HELLO. To verify API token works.')
    parser.add_option('--problem', action='store', type='string', dest='problem', help='Specify a problem.')
    parser.add_option('--file', action='store', type='string', dest='file', help='Specify a file.')
    parser.add_option('--get', action='store_true', help='Get a problem. Needs --problem, too.')
    parser.add_option('--submit', action='store_true', help='submit a problem. Needs --problem and --file, too.')

    options, args = parser.parse_args()

    if options.get and opts.problem is None:
        print('"get" requires "problem"')
        return ({}, [])
    if options.submit and (options.problem is None or options.file is None):
        print('"submit" requires "problem" and "file"')
        return ({}, [])

    return (options, args)


def hello():
    headers = {'Authorization': 'Bearer {}'.format(API_TOKEN)}
    url = 'https://poses.live/api/hello'
    response = requests.get(url, headers=headers)
    print(response.text)
    return 0


def get(problem):
    headers = {'Authorization': 'Bearer {}'.format(API_TOKEN)}
    url = 'https://poses.live/api/problems/{}'.format(problem)
    response = requests.get(url, headers=headers)

    filename = '{}.problem'.forat(problem)
    try:
        with open(filename, 'w') as f:
            f.write(response.text)
            print('Saved problem in "{}"'.format(filename))
    except:
        print('Failed to open file "{}"'.format(filename))

    return 0


def submit(problem, filename):
    headers = {'Authorization': 'Bearer {}'.format(API_TOKEN)}
    url = 'https://poses.live/api/problems/{}/solutions'.format(problem)
    try:
        with open(filename, 'r') as f:
            solution = f.read()
            response = requests.post(url, headers=headers, data=solution)
            print('Trying to submit for problem {}'.format(problem))
            print('Submitted "{}" for {} as ID {}'.format(filename, problem, response.text))
    except:
        print('Failed to open "{}"'.format(filename))

    return 0


def main():
    options, args = get_options()
    if options.hello:
        return hello()
    if options.get:
        return get(options.problem)
    if options.submit:
        return submit(options.problem, options.file)

    print('No workable actions are specified.')


if __name__ == '__main__':
    main()
