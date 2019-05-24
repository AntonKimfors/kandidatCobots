import datetime
import os
from pathlib import Path

dirname = os.path.dirname(__file__)
relativedirname = os.path.join(dirname, '/logs/')
currentDate = datetime.date.today()

my_file = Path("{}{}{}{}".format(dirname, relativedirname, currentDate, '.txt'))
print(my_file)

if not my_file.exists():
    print('hejhej')
    logfile = open(my_file, 'w+')

