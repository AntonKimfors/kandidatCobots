import datetime
import os
from pathlib import Path


def createlog():
    dirname = os.path.dirname(__file__)
    relativedirname = os.path.join(dirname, '/logs/')
    currentDate = datetime.date.today()
    log_dir = Path("{}{}".format(dirname, relativedirname))
    if not log_dir.exists():
        os.makedirs(log_dir)

    my_file = Path("{}{}{}{}".format(
        dirname, relativedirname, currentDate, '.txt'))

    if not my_file.exists():
        print('hejhej')
        logfile = open(my_file, 'w+')


def write_to_log():

    # if not my_file.exists():
    with open(logfile, "a+"):
        # TODO


class LogDoesntExist(Exception):
    def __init__(self, logfile):
        self.logfile = logfile
        
        if not self.logfile.exists():
            print('hejhej')


        if len(self.productOrder) < self.number_of_products:
            self.diffSym = '<'
        else:
            self.diffSym = '>'

    def __str__(self):
        return "The size of the product order does not match the " \
                "given numberOfProducts variable {} {} {}".format(
                    len(self.productOrder), self.diffSym,
                    self.number_of_products)
