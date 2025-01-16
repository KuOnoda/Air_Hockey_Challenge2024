#include <torch/script.h>
#include <iostream>
#include <vector>
#include <sstream>

int main(){

    torch::jit::script::Module model;

    try{
        model = torch::jit::load("/root/rt_manipulators_cpp/samples/test_yamaguchi/model_4states.pt");
    } catch (const c10::Error& e) {
        std::cerr << "failed to load the model";
        return -1;
    }

    std::vector<float> states(4);
    std::string input;

    while(true){
        std::getline(std::cin, input);
        std::istringstream iss(input);

        for(int i = 0; i < 4; ++i){
            int temp;
            if(!(iss >> temp)){
                std::cerr << "error";
            }
            states[i] = static_cast<float>(temp);
        }

        auto states_tensor = torch::from_blob(states.data(), {1,4});

        at::Tensor target_xy = model.forward({states_tensor}).toTensor();

        float target_x = target_xy[0][0].item<float>();
        float target_y = target_xy[0][1].item<float>();

        //std::cout << states[0] << states[1] << states[2] << states[3] << std::endl;
        std::cout << target_x << target_y << std::endl;
    }

    return 0; 
}
