import json

data = '{"title": "Book1", "ISBN": "12345", "author": [{"name": "autho1", "age": 30}, {"name": "autho2", "age": 25}]}'
order = {}
order  = json.loads(data)

print(type(order))
print(order)

for i in range(5):
    print(i)
    i = i+1
    print(i)