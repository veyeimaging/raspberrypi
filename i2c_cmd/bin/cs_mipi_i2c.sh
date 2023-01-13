
#!/bin/sh

<<'COMMENT_SAMPLE'
./cs_mipi_i2c.sh -r -f devid
./cs_mipi_i2c.sh -r -f firmwarever
./cs_mipi_i2c.sh -r -f productmodel
./cs_mipi_i2c.sh -r -f videofmtcap
./cs_mipi_i2c.sh -r -f videofmt
./cs_mipi_i2c.sh -r -f ispcap
./cs_mipi_i2c.sh -r -f powerhz

./cs_mipi_i2c.sh -w -f videofmt -p1 1280 -p2 720 -p3 60
./cs_mipi_i2c.sh -w -f powerhz -p1 50
./cs_mipi_i2c.sh -w -f i2caddr -p1 newaddr


./cs_mipi_i2c.sh -r -f expostate 
./cs_mipi_i2c.sh -r -f wbstate

./cs_mipi_i2c.sh -r -f daynightmode 
./cs_mipi_i2c.sh -w -f daynightmode -p1 0/1/2

./cs_mipi_i2c.sh -r -f hue 
./cs_mipi_i2c.sh -w -f hue -p1 50 

./cs_mipi_i2c.sh -r -f contrast 
./cs_mipi_i2c.sh -w -f contrast -p1 50 

./cs_mipi_i2c.sh -r -f satu 
./cs_mipi_i2c.sh -w -f satu -p1 50 

./cs_mipi_i2c.sh -w -f sysreset
./cs_mipi_i2c.sh -r -f sysreset

./cs_mipi_i2c.sh -w -f expmode -p1 1
./cs_mipi_i2c.sh -r -f expmode

./cs_mipi_i2c.sh -w -f metime -p1 10000
./cs_mipi_i2c.sh -r -f metime

./cs_mipi_i2c.sh -w -f meagain -p1 3 -p2 3
./cs_mipi_i2c.sh -r -f meagain

./cs_mipi_i2c.sh -w -f medgain -p1 3 -p2 3
./cs_mipi_i2c.sh -r -f medgain

./cs_mipi_i2c.sh -w -f awbmode -p1 1
./cs_mipi_i2c.sh -r -f awbmode

./cs_mipi_i2c.sh -w -f mwbgain -p1 0x20 -p2 0x20
./cs_mipi_i2c.sh -r -f mwbgain

./cs_mipi_i2c.sh -w -f mwbcolortemp -p1 3200
./cs_mipi_i2c.sh -r -f mwbcolortemp

./cs_mipi_i2c.sh -w -f paramsave

COMMENT_SAMPLE

I2C_DEV=10;
I2C_ADDR=0x3b;

print_usage()
{
    echo "this shell scripts should be used for CS-MIPI-X!"
	echo "Usage:  ./cs_mipi_i2c.sh [-r/w] [-f] function name -p1 param1 -p2 param2 -p3 param3 -p4 param4 -b bus"
	echo "options:"
	echo "    -r                       read "
	echo "    -w                       write"
	echo "    -f [function name]       function name"
	echo "    -p1 [param1] 			   param1 of each function"
	echo "    -p2 [param2] 			   param2 of each function"
	echo "    -p3 [param3] 			   param3 of each function"
    echo "    -p4 [param4] 		   param4 of each function"
	echo "    -b [i2c bus num] 		   i2c bus number"
    echo "    -d [i2c addr] 		   i2c addr if not default 0x3b"
    echo "support functions: devid,hdver,camcap,firmwarever,productmodel,videofmtcap,videofmt,ispcap,i2caddr,streammode,powerhz,
     daynightmode ,hue ,contrast , satu , expostate , wbstate ,expfrmmode,expmode , aetarget, aetime,aeagc,metime ,meagain , medgain,dmetime ,dmeagain , dmedgain  , awbmode , mwbcolortemp , mwbgain,imagedir,sreg,striggerone,triggeredge,autotgcnt,tgdebncr,tgdly,pickmode,pickone,mipistatus,ledstrobe,slowshuttergain,sysreboot,sysreset,paramsave,yuvseq,i2cwen"
}

######################reglist###################################
########################this is fpga reglist#####################
deviceID=0x00;
HardWare=0x01;
Csi2_Enable=0x03;
CAM_CAP_L=0X04;
CAM_CAP_H=0X05;
I2c_addr=0x06;

StreamMode=0x0E;
SlaveMode=0x0F;

StrobeIO_MODE=0x10;
Strobe_sel=0x11;
Strobe_value=0x12;

TriggerIO_MODE=0x14;
Trigger_sel=0x15;
Trigger_value=0x16;

ExtTrigEdge=0x18;
ExtTrigDebouncerEn=0x19;
ExtTrigDebouncerTimeL=0x1a;
ExtTrigDebouncerTimeM=0x1b;
ExtTrigDebouncerTimeH=0x1c;
SoftTrig=0x1d;
TrigDlyL=0x1e;
TrigDlyM=0x1f;
TrigDlyH=0x20;
TrigDlyE=0x21;
PickModeEnable=0x26;
PickOne=0x27;
YUVSeq=0x28;


AutoTrigCntMaxL=0xC7;
AutoTrigCntMaxM=0XC8;
AutoTrigCntMaxH=0XC9;
AutoTrigEnable=0xCA;

MIPI_COUNT_L=0xCE;
MIPI_COUNT_H=0xCF;
MIPI_STAT=0xD0;
########################this is isp mcu reglist#####################
ARM_VER_L=0x0100;
ARM_VER_H=0x0101;
PRODUCTID_L=0x0102;
PRODUCTID_H=0x0103;
SYSTEM_RESET=0x0104;
PARAM_SAVE=0x0105;
VIDEOFMT_CAP=0x0106; 
VIDEOFMT_NUM=0x0107;
FMTCAP_WIDTH_L=0x0108;
FMTCAP_WIDTH_H=0x0109;
FMTCAP_HEIGHT_L=0x010A;
FMTCAP_HEIGHT_H=0x010B;
FMTCAP_FRAMRAT_L=0x010C;
FMTCAP_FRAMRAT_H=0x010D;

FMT_WIDTH_L=0x0180;
FMT_WIDTH_H=0x0181;
FMT_HEIGHT_L=0x0182;
FMT_HEIGHT_H=0x0183;
FMT_FRAMRAT_L=0x0184;
FMT_FRAMRAT_H=0x0185;
IMAGE_DIR=0x0186;
SYSTEM_REBOOT=0x0187;
NEW_FMT_FRAMRAT_MODE=0x0188;
NEW_FMT_FRAMRAT_L=0x0189;
NEW_FMT_FRAMRAT_H=0x018A;
I2CWEn=0x190;
ISP_CAP_L=0x0200;
ISP_CAP_M=0x0201;
ISP_CAP_H=0x0202;
ISP_CAP_E=0x0203;
POWER_HZ=0x0204;

DAY_NIGHT_MODE=0x0205;

CSC_HUE=0x0206;
CSC_CONTT=0x0207;
CSC_SATU=0x0208;

EXP_FRM_MODE=0x020F;
AE_MODE=0x0210;
EXP_TIME_L=0x0211;
EXP_TIME_M=0x0212;
EXP_TIME_H=0x0213;
EXP_TIME_E=0x0214;

AGAIN_NOW_DEC=0x0215;
AGAIN_NOW_INTER=0x0216;
DGAIN_NOW_DEC=0x0217;
DGAIN_NOW_INTER=0x0218;

AE_SPEED=0x0219;
AE_TARGET=0x021A;
AE_MAXTIME_L=0x021B;
AE_MAXTIME_M=0x021C;
AE_MAXTIME_H=0x021D;
AE_MAXTIME_E=0x021E;
AE_MAXGAIN_DEC=0x021F;
AE_MAXGAIN_INTER=0x0220;

ME_TIME_L=0x0226;
ME_TIME_M=0x0227;
ME_TIME_H=0x0228;
ME_TIME_E=0x0229;
ME_AGAIN_DEC=0x022A;
ME_AGAIN_INTER=0x022B;
ME_DGAIN_DEC=0x022C;
ME_DGAIN_INTER=0x022D;
AE_SLOW_GAIN_DEC=0x22E;
AE_SLOW_GAIN_INTER=0x22F;
AWB_MODE=0x0230;
WB_RGAIN=0x0231;
WB_GGAIN=0x0232;
WB_BGAIN=0x0233;
WB_COLORTEMPL=0x0235;
WB_COLORTEMPH=0x0236;

MWB_COLORTEMPL=0x023A;
MWB_COLORTEMPH=0x023B;

MWB_RGAIN=0x023C;
MWB_GGAIN=0x023D;
MWB_BGAIN=0x023E;

DME_TIME_L=0x0240;
DME_TIME_M=0x0241;
DME_TIME_H=0x0242;
DME_TIME_E=0x0243;
DME_AGAIN_DEC=0x0244;
DME_AGAIN_INTER=0x0245;
DME_DGAIN_DEC=0x0246;
DME_DGAIN_INTER=0x0247;

SNSOR_REG_FLG=0x700;
SNSOR_REG_ADDR_L=0x701;
SNSOR_REG_ADDR_H=0x702;
SNSOR_REG_VAL=0x703;

######################parse arg###################################
MODE=read;
FUNCTION=version;
PARAM1=0;
PARAM2=0;
PARAM3=0;
PARAM4=0;
b_arg_param1=0;
b_arg_param2=0;
b_arg_param3=0;
b_arg_param4=0;
b_arg_functin=0;
b_arg_bus=0;
b_arg_addr=0;

for arg in $@
do
	if [ $b_arg_functin -eq 1 ]; then
		b_arg_functin=0;
		FUNCTION=$arg;
		if [ -z $FUNCTION ]; then
			echo "[error] FUNCTION is null"
			exit;
		fi
	fi
	if [ $b_arg_param1 -eq 1 ] ; then
		b_arg_param1=0;
		PARAM1=$arg;
	fi

	if [ $b_arg_param2 -eq 1 ] ; then
		b_arg_param2=0;
		PARAM2=$arg;
	fi
    if [ $b_arg_param3 -eq 1 ] ; then
		b_arg_param3=0;
		PARAM3=$arg;
	fi
    if [ $b_arg_param4 -eq 1 ] ; then
		b_arg_param4=0;
		PARAM4=$arg;
	fi
	if [ $b_arg_bus -eq 1 ] ; then
		b_arg_bus=0;
		I2C_DEV=$arg;
	fi
    if [ $b_arg_addr -eq 1 ] ; then
		b_arg_addr=0;
		I2C_ADDR=$arg;
	fi
	case $arg in
		"-r")
			MODE=read;
			;;
		"-w")
			MODE=write;
			;;
		"-f")			
			b_arg_functin=1;
			;;
		"-p1")
			b_arg_param1=1;
			;;
		"-p2")
			b_arg_param2=1;
			;;
        "-p3")
			b_arg_param3=1;
			;;
        "-p4")
			b_arg_param4=1;
			;;
		"-b")
			b_arg_bus=1;
			;;
        "-d")
			b_arg_addr=1;
			;;
		"-h")
			print_usage;
			;;
	esac
done
#######################parse arg end########################

#######################Action###############################
if [ $# -lt 1 ]; then
    print_usage;
    exit 0;
fi

pinmux()
{
	sh ./camera_i2c_config >> /dev/null 2>&1
}

read_devid()
{
	local devid=0;
    
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $deviceID );
	devid=$?;
	printf "hardwareid is 0x%2x \n" $devid;
}
read_hdver()
{
    local hardver=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $HardWare );
	hardver=$?;
    printf "hardware logic version is 0x%2x \n" $hardver;
}

read_firmware_ver()
{
	local firmwarever_l=0;
    local firmwarever_h=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ARM_VER_L );
	firmwarever_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ARM_VER_H );
	firmwarever_h=$?;
    printf "r firmware version is %2d.%02d\n" $firmwarever_h $firmwarever_l;
}

read_camcap()
{
    local camcap_l=0;
    local camcap_h=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $CAM_CAP_L );
	camcap_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $CAM_CAP_H );
	camcap_h=$?;
    printf "r camera capbility 0x%02x%02x\n" $camcap_h $camcap_l;
}
read_productmodel()
{
	local productmodel_l=0;
    local productmodel_h=0;
    local productmodel;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $PRODUCTID_L );
	productmodel_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $PRODUCTID_H );
	productmodel_h=$?;
    
    productmodel=$((productmodel_h*256+productmodel_l));
    case $productmodel in
    "55")
        #0x37
        printf "PRODUCT MODEL is CS-MIPI-IMX307\n";
    ;;
    "56")
        #0x38
        printf "PRODUCT MODEL is CS-LVDS-IMX307\n";
    ;;
    "306")
        #0x132
        printf "PRODUCT MODEL is CS-MIPI-SC132\n";
    ;;
    "307")
        #0x133
        printf "PRODUCT MODEL is CS-LVDS-SC132\n";
    ;;
    *)
     printf "r product model is 0x%04x not recognized\n" $productmodel;
     ;;
    esac
}

read_videofmtcap()
{
    local fmtnum=0;
    local width=0;
    local height=0;
    local framerate=0;
    local data_l=0;
    local data_h=0;
	local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $VIDEOFMT_NUM);
	fmtnum=$?;
	printf "camera support videofmt num %d\n" $fmtnum;
    local i=0;
    while [ $i -lt $fmtnum ]
    do
        res=$(./i2c_read $I2C_DEV $I2C_ADDR $(($FMTCAP_WIDTH_L+$i*6)));
        data_l=$?;
        res=$(./i2c_read $I2C_DEV $I2C_ADDR $(($FMTCAP_WIDTH_H+$i*6)));
        data_h=$?;
        width=$((data_h*256+data_l));
        
        res=$(./i2c_read $I2C_DEV $I2C_ADDR $(($FMTCAP_HEIGHT_L+$i*6)));
        data_l=$?;
        res=$(./i2c_read $I2C_DEV $I2C_ADDR $(($FMTCAP_HEIGHT_H+$i*6)));
        data_h=$?;
        height=$((data_h*256+data_l));
        
        res=$(./i2c_read $I2C_DEV $I2C_ADDR $(($FMTCAP_FRAMRAT_L+$i*6)));
        data_l=$?;
        res=$(./i2c_read $I2C_DEV $I2C_ADDR $(($FMTCAP_FRAMRAT_H+$i*6)));
        data_h=$?;
        framerate=$((data_h*256+data_l));
        printf "r videofmtcap num %d width %d height %d framerate %d\n" $(($i+1)) $width $height $framerate;
        i=`expr $i + 1`
    done
    
}

read_videofmt()
{
    local width=0;
    local height=0;
    local framerate=0;
    local newframerate=0;
    local fnewframerate=0;
    local frmratemode=0;
    local data_l=0;
    local data_h=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $FMT_WIDTH_L);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $FMT_WIDTH_H);
	data_h=$?;
    width=$((data_h*256+data_l));
    
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $FMT_HEIGHT_L);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $FMT_HEIGHT_H);
	data_h=$?;
    height=$((data_h*256+data_l));
    
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $NEW_FMT_FRAMRAT_MODE);
	frmratemode=$?;
    if [ $frmratemode -eq 0 ]; then
        res=$(./i2c_read $I2C_DEV $I2C_ADDR  $FMT_FRAMRAT_L);
        data_l=$?;
        res=$(./i2c_read $I2C_DEV $I2C_ADDR  $FMT_FRAMRAT_H);
        data_h=$?;
        framerate=$((data_h*256+data_l));
        printf "r old type videofmt width %d height %d framerate %d\n" $width $height $framerate;
    else
        res=$(./i2c_read $I2C_DEV $I2C_ADDR  $NEW_FMT_FRAMRAT_L);
        data_l=$?;
        res=$(./i2c_read $I2C_DEV $I2C_ADDR  $NEW_FMT_FRAMRAT_H);
        data_h=$?;
        newframerate=$((data_h*256+data_l));
        fnewframerate=$(echo "scale=2;$newframerate/100"|bc);
        printf "r new type videofmt width %d height %d framerate %.2f \n" $width $height $fnewframerate;
    fi
}

write_videofmt()
{
    local width=0;
    local height=0;
    local framerate=0;
    local fnewframerate=0;
    local newframerate=0;
    local frmratemode=0;
    local data_l=0;
    local data_h=0;
    local res=0;
    width=$PARAM1;
    height=$PARAM2;
    framerate=$PARAM3;
    
    # if you use p4, means new video framerate number
    fnewframerate=$PARAM4;
    data_h=$((width/256));
    data_l=$((width%256));
	res=$(./i2c_write $I2C_DEV $I2C_ADDR $FMT_WIDTH_L $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $FMT_WIDTH_H $data_h);
    data_h=$((height/256));
    data_l=$((height%256));
	res=$(./i2c_write $I2C_DEV $I2C_ADDR $FMT_HEIGHT_L $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $FMT_HEIGHT_H $data_h);

    if [ `echo "$fnewframerate > 0" | bc` -eq 1 ]; then
        frmratemode=1;
        res=$(./i2c_write $I2C_DEV $I2C_ADDR $NEW_FMT_FRAMRAT_MODE $frmratemode);
        newframerate=$(echo "$fnewframerate * 100"|bc);
        newframerate=${newframerate%.*};
        #printf "new framerate %d\n" $newframerate;
        data_h=$((newframerate/256));
        data_l=$((newframerate%256));
        res=$(./i2c_write $I2C_DEV $I2C_ADDR $NEW_FMT_FRAMRAT_L $data_l);
        res=$(./i2c_write $I2C_DEV $I2C_ADDR $NEW_FMT_FRAMRAT_H $data_h);
        printf "w new type videofmt width %d height %d framerate %.2f\n" $width $height $fnewframerate;
    else
        frmratemode=0;
        res=$(./i2c_write $I2C_DEV $I2C_ADDR $NEW_FMT_FRAMRAT_MODE $frmratemode);
        data_h=$((framerate/256));
        data_l=$((framerate%256));
        res=$(./i2c_write $I2C_DEV $I2C_ADDR $FMT_FRAMRAT_L $data_l);
        res=$(./i2c_write $I2C_DEV $I2C_ADDR $FMT_FRAMRAT_H $data_h);
        printf "w old videofmt width %d height %d framerate %d\n" $width $height $framerate;
    fi
}

read_ispcap()
{
    local ispcap=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ISP_CAP_L);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ISP_CAP_M);
	data_m=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ISP_CAP_H);
	data_h=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ISP_CAP_E);
	data_e=$?;
    ispcap=$((data_e*256*256*256+data_h*256*256+data_m*256+data_l));
    printf "r ispcap 0x%x\n" $ispcap;
}

write_paramsave()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $PARAM_SAVE 0x1 );
    printf "w paramsave,all param will write to flash\n";
}

read_powerhz()
{
    local powerhz=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $POWER_HZ);
	powerhz=$?;
    printf "r powerhz is %d\n" $powerhz;
}
write_powerhz()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $POWER_HZ $PARAM1 );
    printf "w powerhz %d\n" $PARAM1;
}

write_sysreset()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR $SYSTEM_RESET 0x01 );
    printf "w sysreset,all param will reset\n";
}

read_i2caddr()
{
	local i2caddr=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $I2c_addr );
	i2caddr=$?;
    printf "r i2caddr  0x%2x\n" $i2caddr;
}

write_i2caddr()
{
	local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $I2c_addr $PARAM1 );
    printf "w i2caddr  0x%2x and save\n" $PARAM1;
    I2C_ADDR=$PARAM1
    write_paramsave;
}

read_streammode()
{
	local streammode=0;
    local slavemode=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $StreamMode );
	streammode=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $SlaveMode );
	slavemode=$?;
    
    printf "r streammode 0x%2x slave mode is %d\n" $streammode $slavemode;
}

write_sync_slavemode()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $SlaveMode 0x1 );
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $StrobeIO_MODE 0x0 );
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $Strobe_sel 0x1 );
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $TriggerIO_MODE 0x0 );
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $Trigger_sel 0x2 );
    printf "w stream mode slave \n";
}

write_sync_mastermode()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $SlaveMode 0x0 );
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $StrobeIO_MODE 0x1 );
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $Strobe_sel 0x1 );
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $TriggerIO_MODE 0x1 );
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $Trigger_sel 0x2 );
    printf "w stream mode master \n";
}

write_hardtrigger_mode()
{
    local res=0;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $TriggerIO_MODE 0x0 );
    printf "w stream mode master \n";
}

write_streammode()
{
	local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $StreamMode $PARAM1 );
    if [ $PARAM1 -eq 1 ] ; then
        if [ $PARAM2 -eq 1 ] ; then
            write_sync_slavemode;
        else
            write_sync_mastermode;
        fi
    elif [ $PARAM1 -eq 2 ] ; then
        write_hardtrigger_mode;
    fi
    printf "w streammode 0x%2x slave mode 0x%2x and save param\n" $PARAM1 $PARAM2;
    write_paramsave;
}

read_daynightmode()
{
    local dnmode=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DAY_NIGHT_MODE );
	dnmode=$?;
    printf "r daynight  0x%2x\n" $dnmode;
}

write_daynightmode()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $DAY_NIGHT_MODE $PARAM1 );
    printf "w day night mode  0x%2x \n" $PARAM1;
}

read_hue()
{
    local hue=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $CSC_HUE );
	hue=$?;
    printf "r hue  %d\n" $hue;
}

write_hue()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $CSC_HUE $PARAM1 );
    printf "w hue %d \n" $PARAM1;
}

read_contrast()
{
    local contrast=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $CSC_CONTT );
	contrast=$?;
    printf "r contrast %d\n" $contrast;
}

write_contrast()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $CSC_CONTT $PARAM1 );
    printf "w contrast %d \n" $PARAM1;
}

read_satu()
{
    local saturation=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR  $CSC_SATU );
	saturation=$?;
    printf "r saturation %d\n" $saturation;
}

write_satu()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $CSC_SATU $PARAM1 );
    printf "w saturation  %d \n" $PARAM1;
}

read_expostate()
{
    local exptime=0;
    local again_dec=0;
    local dgain_dec=0;
    local again_int=0;
    local dgain_int=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $EXP_TIME_L);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $EXP_TIME_M);
	data_m=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $EXP_TIME_H);
	data_h=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $EXP_TIME_E);
	data_e=$?;
    exptime=$((data_e*256*256*256+data_h*256*256+data_m*256+data_l));
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AGAIN_NOW_DEC);
	again_dec=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AGAIN_NOW_INTER);
	again_int=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DGAIN_NOW_DEC);
	dgain_dec=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DGAIN_NOW_INTER);
	dgain_int=$?;
    
    printf "r exptime %d us, again %d.%d dB, dgain %d.%d dB \n" $exptime $again_int $again_dec $dgain_int $dgain_dec;
}

read_wbstate()
{
    local rgain=0;
    local ggain=0;
    local bgain=0;
    local colortemp=0;
    local data_l=0;
    local data_h=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $WB_RGAIN);
	rgain=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $WB_GGAIN);
	ggain=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $WB_BGAIN);
	bgain=$?;
    
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $WB_COLORTEMPL);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $WB_COLORTEMPH);
	data_h=$?;
    colortemp=$((data_h*256+data_l));
    printf "r wb state rgain %02x , ggain %02x, bgain %02x  color temperature %d\n" $rgain $ggain $bgain $colortemp;
}

read_expfrmmode()
{
    local expfrmmode=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR $EXP_FRM_MODE);
	expfrmmode=$?;
    printf "r expfrmmode 0x%2x\n" $expfrmmode;
}

write_slowshuttergain()
{
    local agc_dec=0;
    local agc_int=0;
    local res=0;
    agc_int=$PARAM1;
    agc_dec=$PARAM2;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $AE_SLOW_GAIN_DEC $agc_dec);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $AE_SLOW_GAIN_INTER $agc_int);
	printf "w ae slow shutter gain threshold %d.%d dB\n" $agc_int $agc_dec;
}

read_slowshuttergain()
{
    local agc_dec=0;
    local agc_int=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AE_SLOW_GAIN_DEC);
	agc_dec=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AE_SLOW_GAIN_INTER);
	agc_int=$?;
	printf "r ae slow shutter gain threshold %d.%d dB\n" $agc_int $agc_dec;
}

write_expfrmmode()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $EXP_FRM_MODE $PARAM1 );
    printf "w expfrmmode 0x%2x \n" $PARAM1;
}

read_expmode()
{
    local expmode=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR $AE_MODE);
	expmode=$?;
    printf "r expmode 0x%2x\n" $expmode;
}

write_expmode()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AE_MODE $PARAM1 );
    printf "w expmode 0x%2x \n" $PARAM1;
}

read_metime()
{
    local exptime=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ME_TIME_L);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ME_TIME_M);
	data_m=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ME_TIME_H);
	data_h=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ME_TIME_E);
	data_e=$?;
    exptime=$((data_e*256*256*256+data_h*256*256+data_m*256+data_l));
    printf "r mnual exptime %d us\n" $exptime;
}

write_metime()
{
    local exptime=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    exptime=$PARAM1;
    data_e=$((exptime>>24&0xFF));
    data_h=$((exptime>>16&0xFF));
    data_m=$((exptime>>8&0xFF));
    data_l=$((exptime&0xFF));
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $ME_TIME_L $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $ME_TIME_M $data_m);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $ME_TIME_H $data_h);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $ME_TIME_E $data_e);
    printf "w mnual exptime %d us\n" $exptime;
}

read_meagain()
{
    local again_dec=0;
    local again_int=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ME_AGAIN_DEC);
	again_dec=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ME_AGAIN_INTER);
	again_int=$?;
	printf "r manual again %d.%d dB\n" $again_int $again_dec;
}

write_meagain()
{
    local again_dec=0;
    local again_int=0;
    again_int=$PARAM1;
    again_dec=$PARAM2;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $ME_AGAIN_DEC $again_dec);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $ME_AGAIN_INTER $again_int);
	printf "w manual again %d.%d dB\n" $again_int $again_dec;
}

read_medgain()
{
    local dgain_dec=0;
    local dgain_int=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ME_DGAIN_DEC);
	dgain_dec=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ME_DGAIN_INTER);
	dgain_int=$?;
	printf "r manual dgain %d.%d dB\n" $dgain_int $dgain_dec;
}

write_medgain()
{
    local dgain_dec=0;
    local dgain_int=0;
    dgain_int=$PARAM1;
    dgain_dec=$PARAM2;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $ME_DGAIN_DEC $dgain_dec);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $ME_DGAIN_INTER $dgain_int);
	printf "w manual dgain %d.%d dB\n" $dgain_int $dgain_dec;
}

read_dmetime()
{
    local exptime=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DME_TIME_L);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DME_TIME_M);
	data_m=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DME_TIME_H);
	data_h=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DME_TIME_E);
	data_e=$?;
    exptime=$((data_e*256*256*256+data_h*256*256+data_m*256+data_l));
    printf "r direct mnual exptime %d us\n" $exptime;
}

write_dmetime()
{
    local exptime=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    exptime=$PARAM1;
    data_e=$((exptime>>24&0xFF));
    data_h=$((exptime>>16&0xFF));
    data_m=$((exptime>>8&0xFF));
    data_l=$((exptime&0xFF));
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $DME_TIME_L $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $DME_TIME_M $data_m);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $DME_TIME_H $data_h);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $DME_TIME_E $data_e);
    printf "w direct mnual exptime %d us\n" $exptime;
}

read_dmeagain()
{
    local again_dec=0;
    local again_int=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DME_AGAIN_DEC);
	again_dec=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DME_AGAIN_INTER);
	again_int=$?;
	printf "r direct manual again %d.%d dB\n" $again_int $again_dec;
}

write_dmeagain()
{
    local again_dec=0;
    local again_int=0;
    again_int=$PARAM1;
    again_dec=$PARAM2;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $DME_AGAIN_DEC $again_dec);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $DME_AGAIN_INTER $again_int);
	printf "w direct manual again %d.%d dB\n" $again_int $again_dec;
}

read_dmedgain()
{
    local dgain_dec=0;
    local dgain_int=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DME_DGAIN_DEC);
	dgain_dec=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $DME_DGAIN_INTER);
	dgain_int=$?;
	printf "r direct manual dgain %d.%d dB\n" $dgain_int $dgain_dec;
}

write_dmedgain()
{
    local dgain_dec=0;
    local dgain_int=0;
    dgain_int=$PARAM1;
    dgain_dec=$PARAM2;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $DME_DGAIN_DEC $dgain_dec);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $DME_DGAIN_INTER $dgain_int);
	printf "w direct manual dgain %d.%d dB\n" $dgain_int $dgain_dec;
}

read_awbmode()
{
    local awbmode=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR $AWB_MODE);
	awbmode=$?;
    printf "r awbmode 0x%2x\n" $awbmode;
}

write_awbmode()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AWB_MODE $PARAM1 );
    printf "w awbmode 0x%2x \n" $PARAM1;
}

read_mwbcolortemp()
{
    local data_l=0;
    local data_h=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $MWB_COLORTEMPL);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $MWB_COLORTEMPH);
	data_h=$?;
    colortemp=$((data_h*256+data_l));
    printf "r manual wb color temperature %d\n" $colortemp;
}

write_mwbcolortemp()
{
    colortemp=$PARAM1;
    data_h=$((colortemp>>8&0xFF));
    data_l=$((colortemp&0xFF));
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $MWB_COLORTEMPL $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $MWB_COLORTEMPH $data_h);
    printf "w mnualwb color temperature %d \n" $colortemp;
}

read_mwbgain()
{
    local rgain=0;
    #local ggain=0;
    local bgain=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $MWB_RGAIN);
	rgain=$?;
    #res=$(./i2c_read $I2C_DEV $I2C_ADDR  $MWB_GGAIN);
	#ggain=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $MWB_BGAIN);
	bgain=$?;
    printf "r rgain %02x, bgain %02x\n" $rgain $bgain;
}

write_mwbgain()
{
    local rgain=$PARAM1;
    #local ggain=$PARAM2;
    local bgain=$PARAM2;
    local res=0;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $MWB_RGAIN $rgain);
    #res=$(./i2c_write $I2C_DEV $I2C_ADDR  $MWB_RGAIN $ggain);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $MWB_BGAIN $bgain);
    printf "b rgain %02x, bgain %02x\n" $rgain $bgain;
}
read_imagedir()
{
    local imagedir=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR $IMAGE_DIR);
	imagedir=$?;
    printf "r imagedir 0x%2x\n" $imagedir;
}

write_imagedir()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $IMAGE_DIR $PARAM1 );
    printf "w imagedir 0x%2x \n" $PARAM1;
}

read_aetarget()
{
    local aetarget=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR $AE_TARGET);
	aetarget=$?;
    printf "r aetarget 0x%2x\n" $aetarget;
}
write_aetarget()
{
    local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AE_TARGET $PARAM1 );
    printf "w aetarget 0x%2x \n" $PARAM1;
}

read_aetime()
{
    local exptime=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AE_MAXTIME_L);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AE_MAXTIME_M);
	data_m=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AE_MAXTIME_H);
	data_h=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AE_MAXTIME_E);
	data_e=$?;
    exptime=$((data_e*256*256*256+data_h*256*256+data_m*256+data_l));
    if [ $exptime -eq  $((16#FFFFFFFF)) ] ; then
		printf "r auto exptime 0xFFFFFFFF, auto adjust mode\n";
	else
        printf "r auto exptime %d us\n" $exptime;
    fi
}
write_aetime()
{
    local exptime=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    local res=0;
    exptime=$PARAM1;
    data_e=$((exptime>>24&0xFF));
    data_h=$((exptime>>16&0xFF));
    data_m=$((exptime>>8&0xFF));
    data_l=$((exptime&0xFF));
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AE_MAXTIME_L $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AE_MAXTIME_M $data_m);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AE_MAXTIME_H $data_h);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AE_MAXTIME_E $data_e);
    printf "w auto exptime %d us\n" $exptime;
}

read_aeagc()
{
    local agc_dec=0;
    local agc_int=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AE_MAXGAIN_DEC);
	agc_dec=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AE_MAXGAIN_INTER);
	agc_int=$?;
	printf "r ae agc max %d.%d dB\n" $agc_int $agc_dec;
}
write_aeagc()
{
    local agc_dec=0;
    local agc_int=0;
    local res=0;
    agc_int=$PARAM1;
    agc_dec=$PARAM2;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $AE_MAXGAIN_DEC $agc_dec);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $AE_MAXGAIN_INTER $agc_int);
	printf "w ae agc max %d.%d dB\n" $agc_int $agc_dec;
}
write_striggerone()
{
    local res=0;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $SoftTrig 0x1);
	printf "w software trigger one frame!\n";
}

read_senreg()
{
    local senregaddr=0;
    local senregval=0;
    local senregflag=0;
    local data_l=0;
    local data_h=0;
    local res=0;
    senregaddr=$PARAM1;
    data_h=$((senregaddr>>8&0xFF));
    data_l=$((senregaddr&0xFF));
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $SNSOR_REG_FLG 0x0);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $SNSOR_REG_ADDR_L $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $SNSOR_REG_ADDR_H $data_h);
    sleep 0.01;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $SNSOR_REG_FLG);
	senregflag=$?;
     if [ $senregflag -eq  $((16#FF)) ] ; then
		printf "read sensor register failed\n";
	else
        res=$(./i2c_read $I2C_DEV $I2C_ADDR $SNSOR_REG_VAL);
        senregval=$?;
        printf "r sensor addr 0x%4x value 0x%2x\n" $senregaddr $senregval;
    fi
	
}

read_triggeredge()
{
    local triggeredge=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ExtTrigEdge);
	triggeredge=$?;
	printf "r trigger edge %d\n" $triggeredge ;
}

write_triggeredge()
{
    local triggeredge=0;
    local res=0;
    triggeredge=$PARAM1;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $ExtTrigEdge $triggeredge);
	printf "w trigger edge %d\n" $triggeredge ;
}

read_autotgcnt()
{
    local count=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local enable=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AutoTrigCntMaxL);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AutoTrigCntMaxM);
	data_m=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AutoTrigCntMaxH);
	data_h=$?;
    count=$((data_h*256*256+data_m*256+data_l));
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $AutoTrigEnable);
	enable=$?;
    printf "r auto auto trigger count  %d *10us enable %d\n" $count $enable;
}

write_autotgcnt()
{
    local count=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local res=0;
    count=$PARAM1;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AutoTrigEnable 0);
    data_h=$((count>>16&0xFF));
    data_m=$((count>>8&0xFF));
    data_l=$((count&0xFF));
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AutoTrigCntMaxL $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AutoTrigCntMaxM $data_m);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AutoTrigCntMaxH $data_h);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $AutoTrigEnable 1);
    printf "w auto count %d *10us\n" $count;
}
##TrigDebouncer
read_tgdebncr()
{
    local enable=0;
    local value=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ExtTrigDebouncerEn);
	enable=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ExtTrigDebouncerTimeL);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ExtTrigDebouncerTimeM);
	data_m=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $ExtTrigDebouncerTimeH);
	data_h=$?;
    value=$((data_h*256*256+data_m*256+data_l));
    printf "r trigger debouncer enable %d value %d us\n" $enable $value;
}

write_tgdebncr()
{
    local enable=0;
    local value=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local res=0;
    enable=$PARAM1;
    value=$PARAM2;
    
    data_h=$((value>>16&0xFF));
    data_m=$((value>>8&0xFF));
    data_l=$((value&0xFF));
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $ExtTrigDebouncerTimeL $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $ExtTrigDebouncerTimeM $data_m);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $ExtTrigDebouncerTimeH $data_h);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $ExtTrigDebouncerEn $enable);
    
    printf "w trigger debouncer enable %d value %d us\n" $enable $value;
}

read_tgdly()
{
    local triggerdly=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $TrigDlyL);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $TrigDlyM);
	data_m=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $TrigDlyH);
	data_h=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $TrigDlyE);
	data_e=$?;
    triggerdly=$((data_e*256*256*256+data_h*256*256+data_m*256+data_l));
    printf "r triggerdly %d us\n" $triggerdly;
    
}

write_tgdly()
{
    local triggerdly=0;
    local data_l=0;
    local data_m=0;
    local data_h=0;
    local data_e=0;
    local res=0;
    triggerdly=$PARAM1;
    data_e=$((triggerdly>>24&0xFF));
    data_h=$((triggerdly>>16&0xFF));
    data_m=$((triggerdly>>8&0xFF));
    data_l=$((triggerdly&0xFF));
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $TrigDlyL $data_l);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $TrigDlyM $data_m);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $TrigDlyH $data_h);
    res=$(./i2c_write $I2C_DEV $I2C_ADDR  $TrigDlyE $data_e);
    printf "w triggerdly %d us\n" $triggerdly;
}

read_pickmode()
{
    local pickmode=0;
    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR $PickModeEnable);
	pickmode=$?;
	printf "r pickmode %d\n" $pickmode ;
}

write_pickmode()
{
    local pickmode=0;
    local res=0;
    pickmode=$PARAM1;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $PickModeEnable $pickmode);
	printf "w pickmode %d\n" $pickmode ;
}

write_pickone()
{
    local res=0;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $PickOne 1);
	printf " pick mode pick one!\n" ;
}

read_mipistatus()
{
    local mipi_count=0;
    local data_l=0;
    local data_h=0;
    local mipi_stat=0;
    local res=0;

    local res=0;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $MIPI_COUNT_L);
	data_l=$?;
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $MIPI_COUNT_H);
	data_h=$?;
    mipi_count=$((data_h*256+data_l));
    res=$(./i2c_read $I2C_DEV $I2C_ADDR  $MIPI_STAT);
	mipi_stat=$?;
    printf "r mipi count %d, mipi state %d\n" $mipi_count $mipi_stat;
}

write_sysreboot()
{
    local reboot_type=0;
    local res=0;
    reboot_type=$PARAM1;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $SYSTEM_REBOOT $reboot_type);
	printf "w system reboot %d\n" $reboot_type;
}

write_ledstrobe()
{
    local res=0;
    local led_enable=$PARAM1;
    if [ $led_enable -eq 1 ]; then
        res=$(./i2c_write $I2C_DEV $I2C_ADDR  $StrobeIO_MODE 0x1);
        res=$(./i2c_write $I2C_DEV $I2C_ADDR  $Strobe_sel 0x3);
        printf "w led strobe enable\n";
    else
        res=$(./i2c_write $I2C_DEV $I2C_ADDR  $StrobeIO_MODE 0x0);
        res=$(./i2c_write $I2C_DEV $I2C_ADDR  $Strobe_sel 0x0);
        printf "w led strobe disable\n";
    fi
}

read_i2c_write_enable()
{
    local i2cwenable=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR $I2CWEn);
	i2cwenable=$?;
    if [ $i2cwenable -eq 238 ] ; then
        printf "r i2c write enable\n";
    else
		printf "r i2c write disable\n";
	fi
	
}
write_i2c_write_enable()
{
    local i2c_write_enable=0;
	local res=0;
    if [ $PARAM1 -eq 0 ] ; then
        i2c_write_enable=0x11;
    else
		i2c_write_enable=0xEE;
	fi

	res=$(./i2c_write $I2C_DEV $I2C_ADDR $I2CWEn $i2c_write_enable);
    sleep 0.01;
    res=$(./i2c_write $I2C_DEV $I2C_ADDR $I2CWEn $i2c_write_enable);
	printf "w i2c_write_enable is 0x%2x\n" $PARAM1;
}

read_yuvseq()
{
	local yuvseq=0;
	local res=0;
	res=$(./i2c_read $I2C_DEV $I2C_ADDR $YUVSeq );
	yuvseq=$?;
    if [ $yuvseq -eq 1 ] ; then
		printf "r YUVseq is YUYV\n";
    else
        printf "r YUVseq is UYVY\n";
	fi
}

write_yuvseq()
{
	local csienable=0;
	local res=0;
	res=$(./i2c_write $I2C_DEV $I2C_ADDR $YUVSeq $PARAM1);
    if [ $PARAM1 = "YUYV" ] ; then
		res=$(./i2c_write $I2C_DEV $I2C_ADDR  $YUVSeq 0x1);
    else
        res=$(./i2c_write $I2C_DEV $I2C_ADDR  $YUVSeq 0x0);
	fi
	printf "w YUVseq is %s\n" $PARAM1;
}

#######################Action# BEGIN##############################

pinmux;

if [ ${MODE} = "read" ] ; then
	case $FUNCTION in
		"devid"|"deviceid")
			read_devid;
			;;
        "hdver")
			read_hdver;
			;;
        "camcap")
			read_camcap;
			;;
		"firmwarever")
			read_firmware_ver;
			;;
		"productmodel")
			read_productmodel;
			;;
		"videofmtcap")
			read_videofmtcap;
			;;
		"videofmt")
			read_videofmt;
			;;
		"ispcap")
			read_ispcap;
			;;
		"powerhz")
			read_powerhz;
			;;
        "i2caddr")
			read_i2caddr;
			;;
        "streammode")
            read_streammode;
			;;
        "daynightmode")
            read_daynightmode;
			;;
        "hue")
            read_hue;
			;;
        "contrast")
            read_contrast;
			;;
        "satu")
            read_satu;
			;;
        "expostate")
            read_expostate;
			;;
        "wbstate")
            read_wbstate;
			;;
        "expfrmmode")
            read_expfrmmode;
			;;
         "slowshuttergain")
            read_slowshuttergain;
			;;
        "expmode")
            read_expmode;
			;;
        "metime")
            read_metime;
			;;
        "meagain")
            read_meagain;
			;;
        "medgain")
            read_medgain;
			;;
        "dmetime")
            read_dmetime;
			;;
        "dmeagain")
            read_dmeagain;
			;;
        "dmedgain")
            read_dmedgain;
			;;
        "awbmode")
            read_awbmode;
			;;
        "mwbcolortemp")
            read_mwbcolortemp;
			;;
        "mwbgain")
            read_mwbgain;
			;;
        "imagedir")
            read_imagedir;
			;;
        "aetarget")
            read_aetarget;
			;;
        "aetime")
            read_aetime;
			;;
        "aeagc")
            read_aeagc;
			;;
        "sreg")
            read_senreg;
			;;
        "triggeredge")
            read_triggeredge;
			;;
        "autotgcnt")
            read_autotgcnt;
			;;
        "tgdebncr")
            read_tgdebncr;
			;;
        "pickmode")
            read_pickmode;
			;;
        "tgdly")
            read_tgdly;
			;;
        "mipistatus")
            read_mipistatus;
	    		;;
        "i2cwen")
			read_i2c_write_enable;
			;;
        "yuvseq")
            read_yuvseq;
                ;;
        *)
			echo "NOT SUPPORTED!";
			;;
	esac
fi

if [ ${MODE} = "write" ] ; then
	case $FUNCTION in
		"sysreset")
			write_sysreset;
			;;
		"paramsave")
			write_paramsave;
			;;
		"videofmt")
			write_videofmt;
			;;
		"powerhz")
			write_powerhz;
			;;
        "i2caddr")
			write_i2caddr;
			;;
        "streammode")
            write_streammode;
			;;
        "daynightmode")
            write_daynightmode;
			;;
        "hue")
            write_hue;
			;;
        "contrast")
            write_contrast;
			;;
        "satu")
            write_satu;
			;;
        "expfrmmode")
            write_expfrmmode;
			;;
        "slowshuttergain")
            write_slowshuttergain;
			;;
        "expmode")
            write_expmode;
			;;
        "metime")
            write_metime;
			;;
        "meagain")
            write_meagain;
			;;
        "medgain")
            write_medgain;
			;;
        "dmetime")
            write_dmetime;
			;;
        "dmeagain")
            write_dmeagain;
			;;
        "dmedgain")
            write_dmedgain;
			;;
        "awbmode")
            write_awbmode;
			;;
        "mwbcolortemp")
            write_mwbcolortemp;
			;;
        "mwbgain")
            write_mwbgain;
			;;
        "imagedir")
            write_imagedir;
			;;
        "aetarget")
            write_aetarget;
			;;
        "aetime")
            write_aetime;
			;;
        "aeagc")
            write_aeagc;
			;;
        "striggerone")
            write_striggerone;
			;;
        "triggeredge")
            write_triggeredge;
			;;
        "autotgcnt")
            write_autotgcnt;
			;;
        "tgdebncr")
            write_tgdebncr;
			;;
        "pickmode")
            write_pickmode;
			;;
        "pickone")
            write_pickone;
			;;
        "tgdly")
            write_tgdly;
			;;
        "sysreboot")
            write_sysreboot;
			;;
        "ledstrobe")
            write_ledstrobe;
			;;
        "i2cwen")
			write_i2c_write_enable;
			;;
        "yuvseq")
            write_yuvseq;
                ;;
        *)
			echo "NOT SUPPORTED!";
			;;
	esac
fi

