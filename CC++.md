# 指针

## 地址运算符：&
```
#include<iostream>
int main()
{
    using namespace std;
    int donuts=6;
    double cups=4.5;

    cout<<"donuts value = "<<donuts;
    cout<<" and donuts address = "<<&donuts<<endl;
    cout<<"cups value = "<<cups;
    cout<<" and cups address = "<<&cups<<endl;
    return 0;
}
```
运行结果：
```
donuts value = 6 and donuts address = 0x62fe1c
cups value = 4.5 and cups address = 0x62fe10
```
##  *运算符被称为间接值或解除引用运算符 ，将其应用于指针，可以得到该地址处存储的值（这和乘法使用的符号相同，C++根据上下文来确定所指的是乘法还是解除引用）
```
#include <iostream>
int main()
{
    using namespace std;
    int updates=6;
    int *p_updates;
    p_updates=&updates;

    cout<<"Values: updates = "<<updates;
    cout<<", *p_updates = "<<*p_updates<<endl;
    cout<<"Addresses: &updates = "<<&updates;
    cout<<", p_updates = "<<p_updates<<endl;
    *p_updates=*p_updates+1;
    cout<<"Now updates = "<<updates<<endl;
    return 0;
}
```
运行结果：
```
Values: updates = 6, *p_updates = 6
Addresses: &updates = 0x62fe14, p_updates = 0x62fe14
Now updates = 7
```

***
## 使用new来分配内存
对指针的工作方式有一定了解后，来看看它如何实现在程序运行时分配内存。前面我们都将指针初始化为变量的地址；变量是在编译时分配的有名称的内存，而指针只是为可以通过名称直接访问的内存提供了一个别名。指针真正的用武之地在于，在运阶段分配未命名的内存以存储值。在这种情况下，只能通过指针来访问内存。在C语言中，可以用库函数malloco 来分配内存；在C++中仍然可以这样做，但C++还有更好的方法——new运算符。

```int * pn = new int```

 new int 告诉程序，需要适存储int的内存。new运算符根据类型来确定需要多少字节的内存,然后，到这样的内并返其地址
 ```
 #include<iostream>
int main()
{
    using namespace std;
    int nights=1001;
    int *pt=new int;
    *pt=100;
    cout<<"nights value = "<<nights<<": location "<<&nights<<endl;
    cout<<"int value = "<<*pt<<": location "<<pt<<endl;
    double *pd=new double;
    *pd=10000001.0;
    cout<<"double value = "<<*pd<<": location "<<pd<<endl;
    cout<<"location of pointer pd: "<<&pd<<endl;
    cout<<"size of pt = "<<sizeof(pt)<<": size of *pt = "<<sizeof(*pt)<<endl;
    cout<<"size of pd = "<<sizeof(pd)<<": size of *pd = "<<sizeof(*pd)<<endl;
    return 0;
}
```
运行结果：
```
nights value = 1001: location 0x62fe14
int value = 100: location 0x1c7e80
double value = 1e+07: location 0x1c7ea0
location of pointer pd: 0x62fe08
size of pt = 8: size of *pt = 4
size of pd = 8: size of *pd = 8
```
## 将引用用作函数参数
*引用是已定义的变量的别名。* 引用变量的主要用途是用作函数的形参，通过引用变量用作参数，函数将使用原始数据，而不是副本。

*下述语句中的&运算符不是地址运算符*，而是将rodents的变量声明为int &,即指向int变量的引用

```int & rodents=rats```

swapr函数接受两个整数引用作为参数，通过交换引用所指向的变量的值来实现交换。
swapp函数接受两个整数指针作为参数，通过间接访问指针所指向的变量来实现交换。
swapv函数接受两个整数作为参数，通过创建一个临时变量来存储其中一个值，然后交换它们来实现交换。

*swapr函数中的&运算符不是地址运算符*，而是将a,b的变量声明为int &,即指向int变量的引用
```
#include <iostream>
void swapr(int &a, int &b); // a, b are aliases for ints
void swapp(int *p, int *q); // p, q are addresses of ints
void swapv(int a, int b);   // a, b are new variables
int main()
{
    using namespace std;
    int wallet1 = 300;
    int wallet2 = 350;
    cout << "wallet1 = $" << wallet1;
    cout << " wallet2 = $" << wallet2 << endl;
    cout << "Using references to swap contents:\n";
    swapr(wallet1, wallet2); // pass variables
    cout << "wallet1 = $" << wallet1;
    cout << " wallet2 = $" << wallet2 << endl;
    cout << "Using pointers to swap contents again:\n";
    swapp(&wallet1, &wallet2); // pass addresses of variables
    cout << "wallet1 = $" << wallet1;
    cout << " wallet2 = $" << wallet2 << endl;
    cout << "Trying to use passing by value:\n";
    swapv(wallet1, wallet2); // pass values of variables
    cout << "wallet1 = $" << wallet1;
    cout << " wallet2 = $" << wallet2 << endl;
    return 0;
}
void swapr(int & a, int & b) // a、b 是 int 的别名
{
    int temp;
    temp = a; // use a, b for values of variables
    a = b;
    b = temp;
    std::cout << "a = " << a << std::endl;
    std::cout << "b = " << b << std::endl;
    std::cout << "temp = " << temp << std::endl;
    std::cout << "&a = " << &a << std::endl;
    std::cout << "&b = " << &b << std::endl;
}

void swapp(int *p, int *q) // p、q 是整数的地址
{
    int temp;
    temp = *p; // use *p, *q for values of variables
    *p = *q;
    *q = temp;
    std::cout<<"p = " << *p << std::endl;
    std::cout<<"q = " << *q << std::endl;
    std::cout<<"temp = " << temp << std::endl;
}

void swapv(int a, int b) // a, b are new variables
{
    int temp;
    temp = a; // use a, b for values of variables
    a = b;
    b = temp;
    std::cout << "a = " << a << std::endl;
    std::cout << "b = " << b << std::endl;
    std::cout << "temp = " << temp << std::endl;
}
```
运行结果：
```
wallet1 = $300 wallet2 = $350
Using references to swap contents:
a = 350
b = 300
temp = 300
&a = 0x62fe1c
&b = 0x62fe18
wallet1 = $350 wallet2 = $300
Using pointers to swap contents again:
p = 300
q = 350
temp = 350
wallet1 = $300 wallet2 = $350 //结果是对的，将第一个函数交换的值再交换回来
Trying to use passing by value:
a = 350
b = 300
temp = 300
wallet1 = $300 wallet2 = $350 //结果是错误的
```
## ->
箭头成员运算符（->）, 可用于指向结构的指针，就像点运算符可用于结构名一样。例如：如果ps指向一个inflatable结构，则ps->是被指向的结构的price成员。

```
#include<iostream>
struct inflatable
{
    char name[20];
    float volume;
    double price;
};
int main()
{
    using namespace std;
    inflatable *ps=new inflatable;
    cout<<"Enter name of inflatable item: ";
    cin.get(ps->name,20);
    cout<<"Enter volume in cubic feet: ";
    // cin>>(*ps).volume;
    cin>>ps->volume;
    cout<<"Enter price: $";
    cin>>ps->price;
    cout<<"Name: "<<(*ps).name<<endl;
    cout<<"Volume: "<<ps->volume<<" cubic feet\n";
    cout<<"Price: $"<<ps->price<<endl;
    delete ps;
    return 0;
}
```
运行结果：
```
Enter name of inflatable item: Fabulous Frodo
Enter volume in cubic feet: 1.4
Enter price: $27.99
Name: Fabulous Frodo
Volume: 1.4 cubic feet
Price: $27.99
```
