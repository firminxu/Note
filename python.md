# \_\_slots__

类定义时在构造函数前经过 \_\_slow__ 声明后，实例只能使用关键字 \_\_slots__ 中定义或声明的属性 (数据成员) 

https://wiki.python.org/moin/UsingSlots

# \_\_repr__
是Python中的一个特殊方法，用于返回对象的可打印字符串表示形式

为了使用__repr__属性，我们需要在类中定义一个方法，方法名为__repr__，并且接受一个self参数。这个方法应该返回一个字符串，用于表示对象的信息。下面是一个简单的示例：
```
class Person:
    def __init__(self, name, age):
        self.name = name
        self.age = age
    def __repr__(self):
        return f"Person(name='{self.name}', age={self.age})"
```

在这个示例中，我们定义了一个Person类，并在类中定义了__repr__方法。在__repr__方法中，我们使用了字符串插值的方式，将name和age属性的值插入到字符串中。

使用__repr__属性
一旦我们在类中定义了__repr__方法，我们就可以直接使用repr(obj)来获取对象的字符串表示形式。下面是一个简单的示例：​​​​​​​

```
person = Person("Alice", 25)
print(repr(person))  # 输出：Person(name='Alice', age=25)
```
在这个示例中，我们创建了一个Person对象，并使用repr函数来获取它的字符串表示形式。输出结果是"Person(name=‘Alice’, age=25)"。


原文链接：https://blog.csdn.net/Rocky006/article/details/134661519


# \_\_eq__
是一个魔术方法，用于比较两个对象是否相等。当我们使用 == 操作符比较两个对象时，实际上会调用左侧对象的 eq 方法，并将右侧对象作为参数传递给它。如果这个方法返回 True，那么我们就认为这两个对象相等。
下面是一个简单的例子，展示了如何在自定义类中实现 eq 方法：
```
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

p1 = Point(1, 2)
p2 = Point(1, 2)

print(p1 == p2)  # 输出 True

```

# @property
python内置的@property装饰器可以把类的方法伪装成属性调用的方式。即将原来Foo.func()的调用方法，变成Foo.func的方式。
```
	class People:
		def __init__(self, name, age):
			self.name = name
			self.age = age
			
		@property
		def age(self):
			return self.__age
			
		@age.setter
		def age(self,age):
			if isinstance(age,int):
				self.__age = age
			else:
				raise ValueError
				
		@age.deleter
		def age(self):
			print("删除数据")
			
	obj = People("curry", 31)
	print(obj.age)
	obj.age = 19
	print(obj.age)
	del obj.age

```
Python 中 @property 和 @builtins.property 之间的主要区别是：[1]

@property 是一个内置的装饰器，它定义类的属性。 它将方法转换为属性。
@builtins.property 是对builtins 模块中内置属性对象的显式引用。
它们本质上做同样的事情，但是 @builtins.property 更清楚地表明您正在使用内置命名空间中的内置属性对象。

一些关键点：

两者都定义了属性的 getter 方法。 当访问属性时将调用装饰方法。
可以使用 @<property_name>.setter 和 @<property_name>.deleter 语法定义其他设置器和删除器方法。
@property 是简写语法，被转换为使用底层的内置属性对象。
@builtins.property 使用内置显式，有些人认为这是更好的意图文档。
总而言之 - 功能是相同的。 @property 提供了更清晰的语法，而 @builtins.property 则清楚地表明您正在使用内置函数中的内置对象。 这取决于对文档的偏好。

提供的文件中的代码显式使用 @builtins.property 来定义 TransformStamped 消息类的属性。
# @***.setter
以下是如何在 Python 中使用 setter 的示例：
```
class MyClass:
    def __init__(self):
        self._my_var = None

    @property
    def my_var(self):
        return self._my_var

    @my_var.setter
    def my_var(self, value):
        self._my_var = value

```
在此示例中，my_var 是一个私有变量，可以使用 my_var 属性访问和修改。 @property装饰器用于定义getter方法，该方法返回_my_var的值。 @my_var.setter 装饰器用于定义 setter 方法，该方法设置 _my_var 的值。

要设置 my_var 的值，您可以简单地为其分配一个值，如下所示：
```
obj = MyClass()
obj.my_var = 42

```
# @classmethod
Python中的@classmethod装饰器用于为类定义类方法。 类方法是绑定到类本身而不是类的各个实例的方法。 [1]

关于@classmethod需要了解的一些关键事项：

类方法接收类作为第一个参数，而不是像常规方法那样接收类的实例。 按照惯例，第一个参数通常命名为 cls。
当您需要执行与类本身而不是任何单个实例相关的操作时，类方法非常有用。
常见用途包括工厂方法、替代构造函数或修改类状态而不是对象状态的方法。
要定义类方法，请在方法定义之前添加@classmethod。 第一个参数应称为 cls。
```
class Date:
  date_format = "YYYY-MM-DD"

  @classmethod
  def from_string(cls, date_as_string):
    year, month, day = map(int, date_as_string.split('-'))
    date_object = cls(year, month, day) 
    return date_object

```
这里的 from_string 是一个从字符串构造 Date 对象的类方法。

总之，@classmethod 定义了一个绑定到类的方法，接收类作为第一个参数而不是 self/instance，并且对于与类相关的工厂或状态修改方法很有用。
# 函数参数带双星号（**）
使用双星号（**）可以将字典作为关键字参数传递给函数。这样的语法称为“kwargs”或者“关键字参数”。

示例代码如下所示：
```
def my_function(**kwargs):
    for key, value in kwargs.items():
        print("{}: {}".format(key, value))
        
my_dict = {'name': 'Alice', 'age': 25}
my_function(**my_dict)
```
输出结果：
```
name: Alice
age: 25

```
