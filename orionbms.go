package main

import (
	"bytes"
	"fmt"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/mdaffin/go-telegraf"
	"github.com/tarm/serial"
)

type SerialPort interface {
	Read([]byte) (int, error)
	Write([]byte) (int, error)
}

type PID struct {
	Name     string
	Units    string
	PID      int
	OBD2Mode int
	Length   int
	Scaling  float64
	MaxValue float64
	MinValue float64
}

type OrionBMS struct {
	ser              SerialPort
	delay            time.Duration
	buffer           []bytes.Buffer
	PIDs             []PID
	InstallationName string
	SerialNumber     string
}

func NewOrionBMS(s *serial.Port, name string) *OrionBMS {
	serialNum, err := GetSerialNumber(s)
	if err != nil {
		fmt.Errorf("Error getting serial number: %s", err)
	}
	o := OrionBMS{ser: s,
		delay:            150 * time.Millisecond,
		InstallationName: name,
		SerialNumber:     serialNum}
	return &o
}

func extractVoltages(voltages string) []int {
	
	v := make([]int, 0)
	
	for i := 0; i < len(voltages); i = i + 4 {
		p := voltages[i : i+4]
		//fmt.Println(p)
		tmpV, err := strconv.ParseInt(p, 16, 32)
		if err != nil {
			fmt.Errorf("Error parsing int from message: %s", err)
		}
		v = append(v, int(tmpV)/10)
	}
	
	return v
}

func extractTemps(temps string) []int {
	
	t := make([]int, 0)
	
	for i := 0; i < len(temps); i = i + 2 {
		p := temps[i : i+2]
		//fmt.Println(p)
		tmpT, err := strconv.ParseInt(p, 16, 32)
		if err != nil {
			fmt.Errorf("Error parsing int from message: %s", err)
		}
		t = append(t, int(tmpT))
	}
	
	return t
}


func makeMeasurement(voltages []int, start int) []telegraf.Measurement {
	measures := make([]telegraf.Measurement, 0)
	for i, v := range voltages {
		m_point := telegraf.MeasureInt("cell_voltages", "voltage", v).AddTag("cell", strconv.Itoa(start+i+1))
		measures = append(measures, m_point)
	}
	//fmt.Println(measures)
	return measures
}


func makeTempMeasurement(Temps []int, start int) []telegraf.Measurement {
	measures := make([]telegraf.Measurement, 0)
	for i, v := range Temps {
		m_point := telegraf.MeasureInt("BMS_Tmp", "Temp", v).AddTag("Ch", strconv.Itoa(start+i+1))
		measures = append(measures, m_point)
	}
	//fmt.Println(measures)
	return measures
}


func (o *OrionBMS) makeMeasurement(PID string, values []int) []string {
	measurements := make([]string, 0)
	
	var m telegraf.Measurement
	switch PID {
	case "F004":
	case "F006":
	case "F007":
	case "F00A":
	case "F00B":
	case "F00C":
		m = telegraf.MeasureInt("current", "amps", values[0])
	case "F015":
		m = telegraf.MeasureInt("current_unsigned", "amps", values[0])
	case "F00D":
		m = telegraf.MeasureInt("voltage", "voltage", values[0])
	case "F00E":
	case "F00F":
	case "F010":
		m = telegraf.MeasureInt("amphours", "ahr", values[0])
	case "F011":
	case "F012":
	case "F013":
	case "F014":
	case "F018":
	case "F046":
		m = telegraf.MeasureInt("input_voltage", "voltage", values[0])
	case "F200":
		m = telegraf.NewMeasurement("balancing")
		for c, i := range values {
			m.AddInt("rawBytes_"+strconv.Itoa(c), i)
			m.AddInt("balancing_"+strconv.Itoa(c), i>>15)
			m.AddInt("resistance_"+strconv.Itoa(c), ((i << 1) >> 1))
		}
	case "F201":
		m = telegraf.NewMeasurement("balancing")
		for c, i := range values {
			m.AddInt("rawBytes_"+strconv.Itoa(c+12), i)
			m.AddInt("balancing_"+strconv.Itoa(c+12), i>>15)
			m.AddInt("resistance_"+strconv.Itoa(c+12), ((i << 1) >> 1))
		}

	default:
		m = telegraf.MeasureBool("Still Alive", "keep-alive", true)
	}
	m.AddTag("orion_id", o.SerialNumber)
	measurements = append(measurements, m.ToLineProtocal())
	return measurements
}

func (o *OrionBMS) MakeCall(PID string, start int) []string {
	_, err := o.ser.Write([]byte(".:1322" + PID + "\n"))

	if err != nil {
		fmt.Errorf("Error writing to the serial port: %s", err)
	}
	//fmt.Printf("Recieved %d bytes\n", n)

	buf := make([]byte, 128)
	time.Sleep(150 * time.Millisecond)

	_, err = o.ser.Read(buf)
	if err != nil {
		fmt.Errorf("Error reading from the serial port: %s", err)
	}
	buf = bytes.Split(buf, []byte("\n"))[0]
	voltages := strings.Split(string(buf), PID)[1]
	v := extractVoltages(voltages)
	m := makeMeasurement(v, start)
	ms := make([]string, 0)
	for _, v := range m {
		v.AddTag("orion_id", o.SerialNumber)
		ms = append(ms, v.ToLineProtocal())
	}
	return ms
}


func (o *OrionBMS) MakeCallTemp(PID string, start int) []string {
	_, err := o.ser.Write([]byte(".:03"+ PID + "\n"))

	if err != nil {
		fmt.Errorf("Error writing to the serial port: %s", err)
	}
	
	buf := make([]byte, 128)
	time.Sleep(150 * time.Millisecond)
	

	_, err = o.ser.Read(buf)
	if err != nil {
		fmt.Errorf("Error reading from the serial port: %s", err)
	}
	
	buf = bytes.Split(buf, []byte("\n"))[0]

	temperatures := strings.Split(string(buf), ":0662F0FF")[1]
	
	t := extractTemps(temperatures)
	
	//fmt.Println(t)
		
	m := makeTempMeasurement(t, start)
	ms := make([]string, 0)
	
	
	for _, t := range m {
		t.AddTag("orion_id", o.SerialNumber)
		ms = append(ms, t.ToLineProtocal())
	}

	return ms
}

func (o *OrionBMS) readSerialCall(PID string) ([]int, error) {
	buf := make([]byte, 128)
	time.Sleep(o.delay)
	_, err := o.ser.Read(buf)
	if err != nil {
		return nil, fmt.Errorf("Error reading from the serial port: %s", err)
	}
	buf = bytes.Split(buf, []byte("\n"))[0]
	valuesArr := strings.Split(string(buf), PID)[1]
	v := make([]int, 0)

	if PID == "F200" || PID == "F201" {
		for i := 0; i < len(valuesArr); i = i + 4 {
			b := string(valuesArr[i : i+4])
			tmpV, err := strconv.ParseInt(b, 16, 32)
			if err != nil {
				return nil, fmt.Errorf("Error parsing int from message: %s", err)
			}
			v = append(v, int(tmpV))
		}
		return v, nil
	}

	if len(valuesArr) < 2 {
		values64, err := strconv.ParseInt(string(valuesArr), 16, 32)
		v = append(v, int(values64))
		if err != nil {
			return nil, err
		}
	} else if len(valuesArr) < 4 {
		tmpV, err := strconv.ParseInt(string(valuesArr), 16, 32)
		if err != nil {
			return nil, fmt.Errorf("Error parsing int from message: %s", err)
		}
		v = append(v, int(tmpV))
		return v, nil
	} else {

		for i := 0; i < len(valuesArr); i = i + 4 {
			p := string(valuesArr[i : i+4])
			tmpV, err := strconv.ParseInt(p, 16, 32)
			if err != nil {
				return nil, fmt.Errorf("Error parsing int from message: %s", err)
			}
			v = append(v, int(tmpV))
		}
		return v, nil
	}
	return v, nil
}

func (o *OrionBMS) makeSerialCall(PID string) ([]int, error) {
	_, err := o.ser.Write([]byte(".:1322" + PID + "\n"))
	if err != nil {
		fmt.Errorf("Error writing to the serial port: %s", err)
	}
	return o.readSerialCall(PID)
}

func GetSerialNumber(ser *serial.Port) (string, error) {
	_, err := ser.Write([]byte(":02090A\n"))
	if err != nil {
		fmt.Errorf("Error writing to the serial port: %s", err)
	}

	time.Sleep(150 * time.Millisecond)
	buf := make([]byte, 26)
	_, err = ser.Read(buf)
	if err != nil {
		return "", err
	}
	s := string(buf)[9 : len(buf)-1]
	return s, nil
}

func (o *OrionBMS) GetPackCurrentUnsigned() (int, error) {
	pid := "F015"
	v, err := o.makeSerialCall(pid)
	if err != nil {
		return 0, err
	}
	fmt.Println(o.makeMeasurement(pid, v)[0])
	return v[0], nil
}

func (o *OrionBMS) GetPackCurrent() (int, error) {
	pid := "F00C"
	v, err := o.makeSerialCall(pid)
	if err != nil {
		return 0, err
	}
	fmt.Println(o.makeMeasurement(pid, v)[0])
	return v[0], nil
}
func (o *OrionBMS) GetPackVoltage() (int, error) {
	pid := "F00D"
	v, err := o.makeSerialCall(pid)
	if err != nil {
		return 0, err
	}
	fmt.Println(o.makeMeasurement(pid, v)[0])
	return v[0], nil
}

func (o *OrionBMS) GetStateOfCharge() (int, error) {
	pid := "F00F"
	v, err := o.makeSerialCall(pid)
	if err != nil {
		return 0, err
	}
	o.makeMeasurement(pid, v)
	return v[0], nil
}

func (o *OrionBMS) GetBalancing() (int, error) {
	pid := "F200"
	v, err := o.makeSerialCall(pid)
	if err != nil {
		return 0, err
	}
	fmt.Println(o.makeMeasurement(pid, v)[0])

	pid = "F201"
	v, err = o.makeSerialCall(pid)
	if err != nil {
		return 0, err
	}
	fmt.Println(o.makeMeasurement(pid, v)[0])
	return v[0], nil
}

func (o *OrionBMS) GetPackAmphours() (int, error) {
	pid := "F010"
	v, err := o.makeSerialCall(pid)
	if err != nil {
		return 0, err
	}
	fmt.Println(o.makeMeasurement(pid, v)[0])
	return v[0], nil
}

func (o *OrionBMS) GetInputVoltage() (int, error) {
	return 0, nil
}

func (o *OrionBMS) GetOpenVoltage() (int, error) {
	pid := "F010"
	v, err := o.makeSerialCall(pid)
	if err != nil {
		return 0, err
	}
	fmt.Println(o.makeMeasurement(pid, v)[0])
	return v[0], nil
}
func (o *OrionBMS) GetPackSummedVoltage() (int, error) {
	return 0, nil
}

var port string

func main() {
	if len(os.Args) > 1 {
		port = os.Args[1]
	} else {
		port = "/dev/ttyUSB0"
	}
	c := serial.Config{Name: port, Baud: 9600, Parity: serial.ParityEven, ReadTimeout: time.Second * 1}
	s, err := serial.OpenPort(&c)
	if err != nil {
		fmt.Errorf("Error opening serial port: %s", err)
	}
	o := NewOrionBMS(s, "UEP_SmallCube")
	
	m := o.MakeCall("F100", 0)
	for _, v := range m {
		fmt.Println(v)
	}
	

	m = o.MakeCall("F101", 12)
	for _, v := range(m){
		fmt.Println(v)
	}
	
	k := o.MakeCallTemp("22F0FF", 0)
	for _, t := range k {
		fmt.Println(t)
	}

	
	_, err = o.GetPackVoltage()
	if err != nil {
		fmt.Errorf("Error getting pack voltage")
		return
	}

/*
	_, err = o.GetPackAmphours()
	if err != nil {
		fmt.Errorf("Error getting pack amphours")
		return
	}

	_, err = o.GetPackCurrent()
	if err != nil {
		fmt.Errorf("Error getting pack voltage")
		return
	}
*/
	_, err = o.GetPackCurrentUnsigned()
	if err != nil {
		fmt.Errorf("Error getting pack voltage")
		return
	}
	/*
	_, err = o.GetBalancing()
	if err != nil {
		fmt.Errorf("Error Getting Balancing")
	}
	*/
}
