# Project Title

Load testing for the localiser in S901

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
$ sudo apt install python-dev & sudo apt install python3-dev

```

### Installing

Install python modules (eg into a virtual environment)

```
$ pip install -r requirements.txt
```

Source environment variables

```
$ source env.sh
```

## Running the tests

Start locust web server

```
$ cd testing
$ locust
```

In a browser, go to http://127.0.0.1:8089/

Host is the server to test (the localiser). In the Host field, enter http://172.19.232.11:8080 (or whichever IP and port the localiser is running on)


## Deployment

Add additional notes about how to deploy this on a live system


## Authors

* **Jenna Riseley** - [PurpleBooth](https://github.com/PurpleBooth)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details




