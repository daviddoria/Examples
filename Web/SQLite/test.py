import dbm

# Open database, creating it if necessary.
db = dbm.open('test.db')

for k, v in db.iteritems():
    print(k, '\t', v)

# Close when done.
db.close()