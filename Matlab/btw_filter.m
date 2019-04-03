% function [ output_args, output_btw ] = btw_filter(btw_value , input_args )
%     btw_value(1) = btw_value(2);
%     btw_value(2) = (2.302372204611830009e-2* input_args);
%     + (0.95395255590776339982 * btw_value(1));
%     output_args =  (btw_value(2) + btw_value(1));
%     output_btw = btw_value;
% end


% 
% function [ output_args, output_btw ] = btw_filter(btw_value , input_args )
%     btw_value(1) = btw_value(2);
%     btw_value(2) = btw_value(3);
%     
%     btw_value(3) = (5.371697748120518590e-4 * input_args) + (-0.93552890497917862156 * btw_value(1))+(1.93338022587993041412 * btw_value(2));
%     output_args =  (btw_value(1) + btw_value(3)) + 2*btw_value(2);
%     output_btw = btw_value;
% end

%400Hz CF=4Hz, 100Hz CF=1Hz
function [ output_args, output_btw ] = btw_filter(btw_value , input_args )
    btw_value(1) = btw_value(2);
    btw_value(2) = btw_value(3);
    
    btw_value(3) = (9.446918438401618072e-4 * input_args) + (-0.91497583480143362955 * btw_value(1))+(1.91119706742607298189  * btw_value(2));
    output_args =  (btw_value(1) + btw_value(3)) + 2*btw_value(2);
    output_btw = btw_value;
end

% % 100Hz CF=4Hz
% function [ output_args, output_btw ] = btw_filter(btw_value , input_args )
%     btw_value(1) = btw_value(2);
%     btw_value(2) = btw_value(3);
%     
%     btw_value(3) = (1.335920002785651040e-2 * input_args) + (-0.70089678118840259557 * btw_value(1))+(1.64745998107697655399  * btw_value(2));
%     output_args =  (btw_value(1) + btw_value(3)) + 2*btw_value(2);
%     output_btw = btw_value;
% end



% public:
% 		FilterBuLp2()
% 		{
% 			v[0]=0.0;
% 			v[1]=0.0;
% 		}
% 	private:
% 		float v[3];
% 	public:
% 		float step(float x) //class II 
% 		{
% 			v[0] = v[1];
% 			v[1] = v[2];
% 			v[2] = (5.371697748120518590e-4 * x)
% 				 + (-0.93552890497917862156 * v[0])
% 				 + (1.93338022587993041412 * v[1]);
% 			return 
% 				 (v[0] + v[2])
% 				+2 * v[1];
% 		}


% float step(float x) //class II 
% 		{
% 			v[0] = v[1];
% 			v[1] = (9.976493397724665035e-1 * x)
% 				 + (0.99529867954493267401 * v[0]);
% 			return 
% 				 (v[1] - v[0]);
% 		}