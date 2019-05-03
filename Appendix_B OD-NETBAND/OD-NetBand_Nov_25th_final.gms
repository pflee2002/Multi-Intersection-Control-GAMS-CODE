*This GAMS code is to demonstrate how to optimize traffic control strategy within the network with four path flows.
*The MILP formulation for OD-NETBAND is programmed according to the literature OD-NETBAND:
*An Approach for Origin Destination Based Network Progression Band Optimization" published in Transportation Research Record
*and the literature Centralized adaptive traffic control strategy design across multiple intersections based on vehicle path flows:
* An approximated Lagrangian decomposition approach”.
*The constraints numbers are defined in the format of Constraints_A_B_C
*in which A is the equation number in the OD-NETBAND: An approach for origin-destination based network progression band optimization;
*B represents the sub-equation in Constraint A;
*C represents the road segment according to the constraints and the example problem described in the 2nd literature.

*Author: Peirong(Slade) Wang (pw685@msstate.edu) Pengfei(Taylor) Li
*Dec-12-2018



*Intersection
set i /1*3/;

*note for specificc OD pair
set j /1*2/;

*number of OD pairs for which dedicated bands are sought
set p /1*2/;

*loop Integer
parameter m(i)/
1 0
2 0
3 0

/;

*lower limit on cycle length
parameter C1;
C1 = 40;

*upper limit on cycle length
parameter C2 ;
C2 = 70;

*Cycle Length
parameter C ;
C = 50;

*signal frequency
parameter z;
z = 0.02;

*ratios of inbound volume to outbound volume of arterial Progression band
parameter k;
k = 1;

*outbound arterial through volume
parameter cap_v;
cap_v = 300;

*inbound arterial through volume
parameter cap_v_bar;
cap_v_bar = 420;

*Outbound Saturation flow rate
parameter cap_s;
cap_s = 1800;

*Inbound Saturation flow rate
parameter cap_s_bar;
cap_s_bar = 1800;

*outbound  Saturation flow rate for specific OD pair
parameter cap_s_o_d;
cap_s_o_d = 1800;

*inbound Saturation flow rate for specific OD pair
parameter cap_s_o_d_bar;
cap_s_o_d_bar = 1800;

*total number of intersections
parameter n;
n = 3;


*volume / Saturation Rate
parameter a(i)/
1  0.17
2  0.17
3  0.17
 /;

*volume / Saturation Rate
parameter a_bar(i)/
1  0.23
2  0.23
3  0
 /;

*weight for yNs,Ne progression cross-bands at section i
parameter a_o_d(i)/
1  0.17
2  0.17
3  0.17
/;

*weight for yNs,Ne,bar progression cross-bands at section i
parameter a_o_d_bar(i)/
1  0.23
2  0.23
3  0.17
 /;

*outbound red time at Si
parameter r(i)/
1  24
2  32
3  26
 /;

*outbound common red time to allow side street movement
parameter cap_r(i)/
1  24
2  6
3  26
 /;

*inbound common red time to allow side street movement
parameter cap_r_bar(i)/
1  24
2  6
3  26
 /;

*time allocate for outbound left turn green at intersection i
parameter L(i)/
1 0
2 18
3 0

/;

*time allocated for outbound (inbound) left turn green at intersection Ni
parameter L_bar(i)/
1 0
2 18
3 0

/;

*inbound red time at Si
parameter r_bar(i)/
1  24
2  32
3  26
/;

*internode offset,time difference between center of ri and nearest center of ri_bar at intersection ni
parameter delta(i)/
1  0
2  10
3  15

 /;

*ratios of outbound traffic volume turned into mainstreet from the side street to outbound arterial through traffic volume
parameter k_o_d(j)/
1  0
2  0
 /;

*ratios of inbound traffic volume turned into mainstreet from the side street to inbound arterial through traffic volume
parameter k_o_d_bar(j)/
1  0
2  0
/;

*distance between Ni and Ni+1 outbound
parameter d(i)/
1  128
2  128
3  128

 /;

*distance between Ni and Ni+1 inbound
parameter d_bar(i)/
1  128
2  128
3  128

 /;

*outbound green time for through traffic at Si
parameter cap_g(i)
/
1  26
2  18
3  26
 /;

*inbound green time for through traffic at Si
parameter cap_g_bar(i)
/
1  26
2  18
3  26
/;

*lower limit on outbound speed at intersection i
parameter e(i)
/
1  51
2  51
3  51
  /;

*lower limit on inbound speed at intersection i
parameter e_bar(i)
/
1  51
2  51
3  51
  /;

*upper limit on outbound speed at intersection i
parameter f(i)
/
1  88
2  88
3  88
  /;

*upper limit on inbound speed at intersection i
parameter f_bar(i)
/
1  88
2  88
3  88
 /;

*lower limit on outbound speed change at intersection i
parameter g(i)
/
1  0
2  0
3  0
/;

*lower limit on inbound speed change at intersection i
parameter g_bar(i)
/
1  0
2  0
3  0
  /;

*upper limit on outbound speed change at intersection i
parameter h(i)
/
1  44
2  44
3  44
  /;

*upper limit on inbound speed change at intersection i
parameter h_bar(i)
/
1  44
2  44
3  44
  /;

*outbound arterial through volume
parameter cap_v_o_d(i)
/
1  300
2  300
3  300
/;

*inbound arterial through volume
parameter cap_v_o_d_bar(i)
/
1  300
2  300
3  420
  /;

*outbound arterial through volume
parameter t(i)/

1 2
2 2
3 2
/;

*inbound arterial through volume
parameter t_bar(i)/
1 2
2 2
3 2/;


*internode offsets (cycles) at intersection Ni (cycles)
parameter theta(i) /

1 0
2 1
3 0/;

*internode offsets (cycles) at intersection Ni (cycles)
parameter theta_bar(i) /

1 0
2 0
3 0/;

*GAMS function: to get index i's value
parameter val_1(i);

val_1(i)= ord(i);

*GAMS function: to get index j's value
parameter val_2(j);

val_2(j)= ord(j);



*Define Variables
variable z_obj;
positive variable b;
positive variable b_bar;
positive variable y_Ns_Ne;
positive variable y_Ns_Ne_bar;

positive variable w_1;
positive variable w_bar_1;
positive variable w_bar_dot_1;
positive variable w_dot_1;

positive variable w_2;
positive variable w_bar_2;
positive variable w_bar_dot_2;
positive variable w_dot_2;

positive variable w_3;
positive variable w_bar_3;
positive variable w_bar_dot_3;
positive variable w_dot_3;

positive variable x_Ns_Ne_1;
positive variable x_Ns_Ne_bar_1;

positive variable x_Ns_Ne_2;
positive variable x_Ns_Ne_bar_2;

positive variable x_Ns_Ne_3;
positive variable x_Ns_Ne_bar_3;





equation obj;
equation constraint_3_1;
equation constraint_3_2;
equation constraint_4;
equation constraint_5;
equation constraint_6;

equation constraint_7_1_1;
equation constraint_7_1_2;
equation constraint_7_1_3;

equation constraint_7_2_1;
equation constraint_7_2_2;
equation constraint_7_2_3;

equation constraint_7_3_1;
equation constraint_7_3_2;
equation constraint_7_3_3;

equation constraint_8_1_1;
equation constraint_8_1_2;

equation constraint_8_2_1;
equation constraint_8_2_2;

equation constraint_9_1_1;
equation constraint_9_1_2;
equation constraint_9_2_1;
equation constraint_9_2_2;

equation constraint_10_1;
equation constraint_10_2;

equation constraint_11_1;
equation constraint_11_2;
equation constraint_11_3;

equation constraint_12_1;
equation constraint_12_2;

equation constraint_13_1;
equation constraint_13_2;

equation constraint_14_1;
equation constraint_14_2;

equation constraint_15_1;
equation constraint_15_2;

equation constraint_16_1;
equation constraint_16_2;
equation constraint_17_1;
equation constraint_17_2;

equation constraint_18;
equation constraint_19;

*Objective Function from OD-Net Band paper equation (1)
obj..z_obj =e= sum((i)$( a(i) and a_bar(i) and val_1(i)<=2),(a(i)*b)+(a_bar(i)*b_bar))+sum((i)$( a_o_d(i)),a_o_d(i)*y_Ns_Ne)+sum((i)$( a_o_d_bar(i)),a_o_d_bar(i)*y_Ns_Ne_bar);

*decomposed from OD-Net Band paper equation (3)
constraint_3_1..1/C2 =l= z;
constraint_3_2..1/C1 =g= z;

* OD-Net Band paper equation (4)
constraint_4..(1-k)*b_bar =g= (1-k)*k*b;
* OD-Net Band paper equation (5)
constraint_5..sum((j)$(k_o_d(j)),(1-k_o_d(j))*y_Ns_Ne) =g= sum((j)$(k_o_d(j)),(1-k_o_d(j))*k_o_d(j)*b);
* OD-Net Band paper equation (6)
constraint_6..sum((j)$(k_o_d_bar(j)),(1-k_o_d_bar(j))*y_Ns_Ne_bar) =g= sum((j)$(k_o_d_bar(j)),(1-k_o_d_bar(j))*k_o_d_bar(j)*b_bar);

*decomposed from OD-Net Band paper equation (7)
constraint_7_1_1..sum((i)$( r(i) and val_1(i)=1),w_1+b-C+r(i)) =l= 0;
constraint_7_1_2..sum((i)$( r(i) and val_1(i)=2),w_2+b-C+r(i)) =l= 0;
constraint_7_1_3..sum((i)$( r(i) and val_1(i)=3),w_3+b-C+r(i)) =l= 0;

constraint_7_2_1..sum((i)$( r_bar(i) and val_1(i)=1),w_bar_1+b_bar-C+r_bar(i)) =l= 0;
constraint_7_2_2..sum((i)$( r_bar(i) and val_1(i)=2),w_bar_2+b_bar-C+r_bar(i)) =l= 0;
constraint_7_2_3..sum((i)$( r_bar(i) and val_1(i)=3),w_bar_3+b_bar-C+r_bar(i)) =l= 0;

constraint_7_3_1..sum((i)$( r_bar(i) and val_1(i)=1),w_bar_dot_1+b_bar-C+r_bar(i)) =l= 0;
constraint_7_3_2..sum((i)$( r_bar(i) and val_1(i)=2),w_bar_dot_2+b_bar-C+r_bar(i)) =l= 0;
constraint_7_3_3..sum((i)$( r_bar(i) and val_1(i)=3),w_bar_dot_3+b_bar-C+r_bar(i)) =l= 0;

*speed is constant in the example problem, so constraint 8 and 9 will not affect the result
*decomposed from OD-Net Band paper equation (8)
*constraint_8_1_1..sum((i)$(d(i) and f(i) and val_1(i) <>3 and t(i) ),(d(i)/f(i))*z - t(i)) =l= 0 ;
*constraint_8_1_2..sum((i)$(d(i) and e(i) and val_1(i) <>3 and t(i) ), t(i) - (d(i)/e(i))*z) =l= 0 ;


*constraint_8_2_1..sum((i)$(d_bar(i) and f_bar(i) and val_1(i) <= 2 and t_bar(i) ),(d_bar(i)/f_bar(i))*z - t_bar(i)) =l= 0;
*constraint_8_2_2..sum((i)$(d_bar(i) and e_bar(i) and val_1(i) <=2 and t_bar(i) ), t_bar(i) - (d_bar(i)/e_bar(i))*z) =l= 0 ;

*decomposed from OD-Net Band paper equation (9)
*constraint_9_1_1..sum((i)$(d(i) and h(i) and  val_1(i) <= 2),((d(i)/h(i))*z - (((d(i)/d(i+1))*t(i+1))-t(i))))=l= 0;
*constraint_9_1_2..sum((i)$(d(i) and g(i) and val_1(i) <= 2), ((d(i)/d(i+1))*t(i+1)-t(i) -((d(i)/g(i))*z))) =l= 0;

*constraint_9_2_1..sum((i)$(d_bar(i) and h_bar(i) and val_1(i) <= 2),(d_bar(i)/h_bar(i))*z - (((d_bar(i)/d_bar(i+1))*t_bar(i+1))-t_bar(i))) =l= 0;
*constraint_9_2_2..sum((i)$(d_bar(i)  and g_bar(i) and  val_1(i) <= 2),((d_bar(i)/d_bar(i+1))*t_bar(i+1))-t_bar(i) - (d_bar(i)/g_bar(i))*z) =l= 0;

*decomposed from OD-Net Band paper equation (10)
constraint_10_1..sum((i)$(L(i) and L_bar(i) and m(i) and r(i) and val_1(i)=1 and theta(i) and t(i) and t_bar(i) and theta_bar(i) ),(w_1+w_bar_1)-(w_2 +w_bar_2)+(t(i)+t_bar(i))+theta(i)*L(i)-theta_bar(i)*L_bar(i)-theta(i+1)*L(i+1)+theta_bar(i+1)*L_bar(i+1)-m(i) -r(i+1)+r(i)) =e= 0 ;
constraint_10_2..sum((i)$(L(i) and L_bar(i) and m(i) and r(i) and val_1(i)=2 and theta(i) and t(i) and t_bar(i) and theta_bar(i) ),(w_2+w_bar_2)-(w_3 +w_bar_3)+(t(i)+t_bar(i))+theta(i)*L(i)-theta_bar(i)*L_bar(i)-theta(i+1)*L(i+1)+theta_bar(i+1)*L_bar(i+1)-m(i) -r(i+1)+r(i)) =e= 0 ;

*decomposed from OD-Net Band paper equation (11)
constraint_11_1..sum((i)$( r_bar(i) and val_1(i) =1),(w_bar_dot_1+b_bar+w_bar_1 - C +r_bar(i)))=e= 0 ;
constraint_11_2..sum((i)$( r_bar(i) and val_1(i) =2),(w_bar_dot_2+b_bar+w_bar_2 - C +r_bar(i)))=e= 0 ;
constraint_11_3..sum((i)$( r_bar(i) and val_1(i) =3),(w_bar_dot_3+b_bar+w_bar_3 - C +r_bar(i)))=e= 0 ;

*decomposed from OD-Net Band paper equation (12)
constraint_12_1..sum((i)$(r_bar(i)  and val_1(i) =1),(r_bar(i)-r_bar(i+1)+w_bar_dot_1-w_bar_dot_2)+w_1-w_2)=e= 0;
constraint_12_2..sum((i)$(r_bar(i)  and val_1(i) =2),(r_bar(i)-r_bar(i+1)+w_bar_dot_2-w_bar_dot_2)+w_2-w_3)=e= 0;

*decomposed from OD-Net Band paper equation (13)
constraint_13_1..sum((i)$(r(i)and val_1(i)=2),x_Ns_Ne_2 -r(i)) =g= 0;
constraint_13_2..sum((i)$(r(i)and val_1(i)=3),x_Ns_Ne_3 -r(i)) =g= 0;

*decomposed from OD-Net Band paper equation (14)
constraint_14_1..sum((i)$( r_bar(i)and val_1(i)=2),x_Ns_Ne_bar_2 -r_bar(i)) =g= 0;
constraint_14_2..sum((i)$( r_bar(i)and val_1(i)=1),x_Ns_Ne_bar_1 -r_bar(i)) =g= 0;

*decomposed from OD-Net Band paper equation (15)
constraint_15_1..sum((i)$( r(i) and val_1(i)= 1),y_Ns_Ne+x_Ns_Ne_1 - r(i)) =l= 0 ;
constraint_15_2..sum((i)$( r_bar(i)and val_1(i)= 3),y_Ns_Ne_bar+x_Ns_Ne_bar_3 - r_bar(i)) =l= 0 ;

*decomposed from OD-Net Band paper equation (16)
constraint_16_1..sum((i)$(  r(i) and val_1(i)=1 ),x_Ns_Ne_1-x_Ns_Ne_2 -w_1 + w_2 + r(i+1) - r(i)) =e= 0;
constraint_16_2..sum((i)$(  r(i) and val_1(i)=2 ),x_Ns_Ne_2-x_Ns_Ne_3 -w_2 + w_3 + r(i+1) - r(i)) =e= 0;

*decomposed from OD-Net Band paper equation (17)
constraint_17_1..sum((i)$( r_bar(i) and val_1(i)=1 ),x_Ns_Ne_bar_2-x_Ns_Ne_bar_1 - w_bar_dot_2+ w_bar_dot_1 - r_bar(i+1)+ r_bar(i)) =e= 0;
constraint_17_2..sum((i)$( r_bar(i) and val_1(i)=2 ),x_Ns_Ne_bar_3-x_Ns_Ne_bar_2 - w_bar_dot_3+ w_bar_dot_2 - r_bar(i+1)+ r_bar(i)) =e= 0;

*OD-Net Band paper equation (18)
constraint_18..sum((i)$(r(i)),y_Ns_Ne+b-C+r(i)) =l= 0;

*OD-Net Band paper equation (19)
constraint_19..sum((i)$(r_bar(i)),y_Ns_Ne_bar+b_bar-C+r_bar(i)) =l= 0;



*GAMS thread option
option threads=12;
model vehicle_optimization
/
obj,
constraint_3_1,
constraint_3_2,
constraint_4,
constraint_5,
constraint_6,

constraint_7_1_1,
constraint_7_1_2,
constraint_7_1_3,

constraint_7_2_1,
constraint_7_2_2,
constraint_7_2_3,

constraint_7_3_1,
constraint_7_3_2,
constraint_7_3_3,

*constraint_8_1_1,
*constraint_8_1_2,

*constraint_8_2_1,
*constraint_8_2_2,

*constraint_9_1_1,
*constraint_9_1_2,
*constraint_9_2_1,
*constraint_9_2_2,

constraint_10_1,
constraint_10_2,

constraint_11_1,
constraint_11_2,
constraint_11_3,

constraint_12_1,
constraint_12_2,

constraint_13_1,
constraint_13_2,

constraint_14_1,
constraint_14_2,

constraint_15_1,
constraint_15_2,

constraint_16_1,
constraint_16_2,
constraint_17_1,
constraint_17_2,

constraint_18,
constraint_19,


/;
solve vehicle_optimization using MIP maximizing z_obj ;




display z_obj.l;
display b.l;
*display b_bar.l;
*display y_Ns_Ne.l;
display y_Ns_Ne_bar.l;
*display w_1.l;
*display w_2.l;
*display w_3.l;
*display w_bar_1.l;
*display w_bar_2.l;
*display w_bar_3.l;
*display w_bar_dot_1.l;
*display w_bar_dot_2.l;
*display w_bar_dot_3.l;
*display x_Ns_Ne_1.l;
*display x_Ns_Ne_2.l;
*display x_Ns_Ne_3.l;
*display x_Ns_Ne_bar_1.l;
*display x_Ns_Ne_bar_2.l;
*display x_Ns_Ne_bar_3.l;
