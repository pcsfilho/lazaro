class Stack:
  def __init__(self):
      self.items = []

  def isEmpty(self):
      return self.items == []

  def push(self, item):
    if not self.contains(item):
      self.items.append(item)

  def pop(self):
    return self.items.pop()

  def peek(self):
    return self.items[len(self.items)-1]

  def contains(self, item):
    return item in self.items
  
  def size(self):
      return len(self.items)
      