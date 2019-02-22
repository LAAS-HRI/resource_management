//TODO: includes

int main(int argc, char *argv[])
{
    CoordinationSignals<BoolCoordMsg>  coordSwitch("on_off_coordination");
    CoordinationSignals<ColorCoordMsg> coordColor("color_coordination");

    ReactiveInputs<BoolMsg> switchOnOff("onoff");
    ReactiveInputs<ColorMsg> emotion("emotion");
    
    ResourceManager mgr({coordSwitch,coordColor},{switchOnOff,emotion});

    mgr.loadControllerPlugin(/*TODO: this is a parameter of the node*/"/path/to/plugin");

    mgr.run();
}
