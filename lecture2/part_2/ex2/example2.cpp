#include <iostream>
class Car
{
public:
    Car();
    ~Car();

    void run();

private:
//私有变量在类的外部无法被访问
    int count_;

};
Car::Car()
{
    std::cout << "Car 构造完成" << std::endl;
    count_=0;
}

Car::~Car()
{
    std::cout << "Car 析构完成" << std::endl;
}

int main()
{
    {
        Car car;
        for(int i=0;i<10;i++){
            car.run();
        }
    }


    return 0;
}
void Car::run(){
    count_++;
    std::cout << "car 使用次数："<<count_ << std::endl;
}