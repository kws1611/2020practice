void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

double quat_mult[4] (double a_1,double a_2,double a_3,double a_4,double b_1,double b_2,double b_3,double b_4){
  /*
  double q_0 = a_1*b_1 - a_2*b_2 - a_3*b_3 - a_4*b_4;
  double q_1 = a_1*b_2 + a_2*b_1 + a_3*b_4 - a_4*b_3;
  double q_2 = a_1*b_3 - a_2*b_4 + a_3*b_1 + a_4*b_2;
  double q_3 = a_1*b_4 + a_2*b_3 - a_3*b_2 + a_4*b_1;
  */
  double quat_mult[0] = a_1*b_1 - a_2*b_2 - a_3*b_3 - a_4*b_4;
  double quat_mult[1] = a_1*b_2 + a_2*b_1 + a_3*b_4 - a_4*b_3;
  double quat_mult[2] = a_1*b_3 - a_2*b_4 + a_3*b_1 + a_4*b_2;
  double quat_mult[3] = a_1*b_4 + a_2*b_3 - a_3*b_2 + a_4*b_1;

  return {q_0, q_1, q_2, q_3};
}
