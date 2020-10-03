test = "r44/l3"

test = test.split("/")

for i in range(len(test)):
    print(int(test[i][1:]))
    print(type(int(test[i][1:])))
