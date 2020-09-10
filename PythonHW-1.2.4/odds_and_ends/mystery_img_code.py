from pathlib import Path
from itertools import chain
from collections import Counter
path_to_file = Path("resources/example-survey.txt")
with open(path_to_file, mode="r") as my_open_file:
    out = [line.strip() for line in my_open_file]
new = list(chain.from_iterable([[a, c] for a, b, c in (x.partition(',') for x in out)]))
while("" in new): 
    new.remove("")
Count = Counter(new)
categories = list(chain.from_iterable([[a, c] for a, b, c in (x.partition(':') for x in new)]))
categories1 = []
items = []
for i in range(len(categories)-1):
    if i%2 == 1:
        categories1.append(categories[i])
    else:
        items.append(categories[i])
categories_clean = []
for i in categories1: 
    if i not in categories_clean: 
        categories_clean.append(i)
##Output = {categories_clean}
print(Count)