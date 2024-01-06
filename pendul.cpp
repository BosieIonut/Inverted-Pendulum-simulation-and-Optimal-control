#include <SFML/Graphics.hpp>
#include <math.h>
// #define cart_x 100.f
// #define cart_y 75.f

// #define L 200
#define screen_x 900
#define screen_y 600
#define freq 300.f
class CartPend
{
    private:
        float dim_cart_x,dim_cart_y,pend_thickness; // cacateristici cart
        float m,M,g,d, gamma,L, Ts  = 1.f / freq; // caract fizice
        float x[4] , dx[4]; // state
        float Sx,Cx,a2,b1,b2,c1,c2,miu; //aux
        sf::RectangleShape pend;
        sf::RectangleShape cart;
    public:
        CartPend()
        {
            dim_cart_x = 100.f;
            dim_cart_y = 75.f;
            L = 2.f;
            pend_thickness = 3.f;
            m = 1.f;
            M = 5.f;
            g = 9.81f;
            d = 1.f;
            gamma = 1.f;
            x[0] = 1.f;
            x[1] = 0.f;
            x[2] = 2.9f;
            x[3] = 0.f;

            //cart_pend sizes (should make a function)
            pend.setSize(sf::Vector2f(100.f * L, 1.f));
            pend.setOutlineThickness(pend_thickness);
            cart.setSize(sf::Vector2f(dim_cart_x,dim_cart_y));

            //set color
            set_color_pend(sf::Color(10.f,20.f,250.f));
            set_color_cart(sf::Color::Green);

            
            set_position(0,0);
        }
        void set_color_cart(sf::Color col)
        {
            cart.setFillColor(col);  
        }
        void set_color_pend(sf::Color col)
        {
            pend.setOutlineColor(col);
            pend.setFillColor(col);
        }
        void set_position(int x,int theta)
        {
         // x can be [ -screen_x / 2 ; screen_x / 2]
         // 0 means center
         // theta = 0 pendulum is up
         x = screen_x / 2 + x ;
         theta = theta - 90;
         cart.setPosition(x - dim_cart_x / 2, screen_y / 2);
         pend.setPosition(x, screen_y / 2 );
         pend.setRotation(-theta);
        }
        sf::RectangleShape *get_cart()
        {
            return &cart;
        }
        sf::RectangleShape *get_pend()
        {
            return &pend;
        }
        float get_angle()
        {
            return x[2];
        }
        float get_pos()
        {
            return x[0];
        } 
        float *get_output()
        {
            return x;
        }
        void compute_dx(float u, float disturbance)
        {
            Sx = sin(x[2]);
            Cx = cos(x[2]); 
            // a1 = (M+m)
            b1 = m * L * Cx;
            c1 = disturbance + u + m * L * x[3] * x[3] * Sx - d * x[1];
            a2 = m * L * Cx;
            b2 = (0.06 + m * L * L);
            c2 = - m * L * g * Sx - gamma * x[3];
            miu = -(M+m) / a2;
            b1 = b1 +  miu * b2;
            c1 = c1 + miu * c2;
            
            dx[3] = c1/b1;
            dx[1] = (c2 - b2 * dx[3]) / a2;
            dx[0] = x[1];
            dx[2] = x[3];

        }
        void compute_next_pos(float u, float disturbance)
        {
            compute_dx(u,disturbance);
            
            x[0] = x[0] +  Ts * dx[0];
            x[1] = x[1] +  Ts * dx[1];
            x[2] = x[2] +  Ts * dx[2];
            x[3] = x[3] +  Ts * dx[3];
            set_position(x[0] * 100,x[2] * 360 / 6.2831853);
        }


};



class PID_Controller
{
    private:
    float int_e,de,u,Ts,last_e;
    float Kp,Ki,Kd;
    public:
    PID_Controller()
    {
        Kp = 20;
        Ki = 40;
        Kd = 0.5;
        int_e = 0;
        Ts = 1.0f / freq;
        last_e = 0;
    }

    float command_compute(float e)
    {
        int_e = int_e + e;
        de = (e - last_e ) / Ts;
        
        u = Kp * e + Ki * int_e * Ts + Kd * de;

        return u;
    }
};

class Kalmann_controller
{
    float vector_multiply(float *A, float *B, int n)
    {
        float sum = 0;
        for(int i = 0 ; i < n ; i++ )
        {
            sum = sum + A[i] * B[i];
        }
        return sum;
    }
    
    float A[4][4] ={ {-0.00127182175503700,	0.00166709061651381,	3.94235186800716e-05,	-3.41392103651654e-05},
                   {-0.822325710567857,	1.0016835394447,	-0.0273696005102993,	-0.0411143107014572},
                   {0.000124777645149689,	1.46087114877615e-06,	-0.00119350984522920,	0.00164714634719944},
                   {0.0751123109316064,	0.00162927913251172,	-0.800751050735204,	0.977619526092949}};

    float B[4][2] = {
    { 1.00127237022500, -0.000119163068751687},
    {0.8229842408556930, -0.0683721219973287},
    {-0.000124507489320891,1.00116763498117},
    {-0.0747879941623406, 0.769690819410787},
    };
   





 
    float C[4] = {0.991773900264344,	5.60122744236591,	-153.853571251033,	-61.6150928258049};
    float x[4] = {0,0,0,0};
    float y[2] = {0,0};
    public:
    float command_compute(float y1,float y2)
    {
        y[0] = y1;
        y[1] = y2;
        // x_k+1 = A*x + b*u;
        // y = C * x;
        printf("%f\n",vector_multiply(A[0],A[1],4));
        for (int i = 0 ; i < 4 ; i++)
            x[i] = vector_multiply(A[i],x,4) + vector_multiply(B[i],y,2);

        return vector_multiply(C,x,4);
    }

 
};
int main()
{

    sf::RenderWindow window(sf::VideoMode(screen_x, screen_y), "Pendulum go brr!!!");
    window.setFramerateLimit(300);
    //95.6659,	195.653,	-1532.279,	-660.5643
    //29.8322,   64.6938, -579.4701, -246.2278
    //29.635,	129.27,	-1328.45,	-642.69
    ///297.77,	394.45,	-2594.17,	-1154.425
    CartPend cart_pend;;
    Kalmann_controller kalman;
    float F[4] = {297.77,	394.45,	-2594.17,	-1154.425},fps,*x,u;
    sf::Clock clock,clock2;
    float time = 0,disturbance;
    system("sleep 5");
    u = 0;
    while (window.isOpen())
    {
        fps = 1.f / clock.restart().asSeconds();

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        x = cart_pend.get_output();
        u = F[0] * x[0] + F[1] * x[1] + F[2] * (x[2] - 3.1416) + F[3] * x[3];
        //u = kalman.command_compute(x[0],(x[2] - 3.14159 ));
        time = clock2.getElapsedTime().asSeconds();
        if(time > 10 && time < 20)
        {
            disturbance = 500;
        }
        else
        {
            disturbance = 0;
        }
        cart_pend.compute_next_pos(u,disturbance); 

        window.clear();
        window.draw( *cart_pend.get_cart());
        window.draw( *cart_pend.get_pend());
        window.display();
    }

    return 0;
}


