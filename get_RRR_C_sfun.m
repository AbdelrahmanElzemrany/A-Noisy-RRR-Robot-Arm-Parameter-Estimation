function get_RRR_C_sfun(block)
    % Setup the Level-2 S-Function block properties
    setup(block);
end

function setup(block)
    % Register 2 input ports and 1 output port
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 1;
    
    % Configure Input Port 1 (in1: 3x1 joint position vector)
    block.InputPort(1).Dimensions        = [3 1]; 
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(1).DataTypeId        = 0; % double
    
    % Configure Input Port 2 (in2: 3x1 joint velocity vector)
    block.InputPort(2).Dimensions        = [3 1]; 
    block.InputPort(2).DirectFeedthrough = true;
    block.InputPort(2).DataTypeId        = 0; % double
    
    % Configure Output Port (EstTau_C: 3x1 Coriolis torque vector)
    block.OutputPort(1).Dimensions       = [3 1]; 
    block.OutputPort(1).DataTypeId       = 0; % double
    
    % Inherit sample time from the model
    block.SampleTimes = [-1 0];
    
    % Standard simulation state optimization flag
    block.SimStateCompliance = 'DefaultSimState';
    
    % Register the calculation callback
    block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
    % 1. Read 6x1 vectors from the inputs
    in1 = block.InputPort(1).Data;
    in2 = block.InputPort(2).Data;
    
    % 2. Calculate the Coriolis torques using your generated code
    EstTau_C = get_RRR_C(in1, in2);
    
    % 3. Send the 6x1 output vector to Simulink
    block.OutputPort(1).Data = EstTau_C;
end
