

class Driver{
public:	
Driver() {init();}

	int getRPS() {return rws;}
	int getLPS() {return lws;}
	void setRWS(int RWS);
	void setLWS(int LWS);
	
	void turn_correction(float error, float PID);
	void manual();
	void turn_correction(float angle);
	
private:
	void init();

	int rws;
	int lws;

	void forward();
	void reverse();

	void pivot_right();
	void pivot_left();
	
	void stop();

	
	const int RW1 = 0; const int RW2 = 1; const int RPWM = 24;
	const int LW1 = 3; const int LW2 = 2; const int LPWM = 25;

};






