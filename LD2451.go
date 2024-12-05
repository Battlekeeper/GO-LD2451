package LD2451

import (
	"bytes"
	"time"

	"github.com/tarm/serial"
)

type Config struct {
	SerialPort       string
	BaudRate         int
	TargetBufferSize int //Size of the channel buffer to store targets in
}

type Target struct {
	Alarm     bool      // Alarm state
	Angle     int       // Angle of the target relative to the perpendicular direction of the antenna
	Distance  int       // Distance in meters to the target
	Direction Direction // Direction of movement relative to the antenna
	Speed     int       // Speed in KM/H
	SNR       int       // Signal to Noise Ratio
}

const (
	DirectionAway   Direction = 0
	DirectionToward Direction = 1
)

type Direction int

func (d Direction) String() string {
	switch d {
	case DirectionAway:
		return "Away"
	case DirectionToward:
		return "Toward"
	default:
		return "Unknown"
	}
}

type LD2451 struct {
	config  Config
	targets chan Target
	errors  chan error
	port    *serial.Port
}

var (
	frameheader = []byte{0xf4, 0xf3, 0xf2, 0xf1}
)

func Open(config Config) (*LD2451, error) {
	serialConfig := &serial.Config{
		Name:        config.SerialPort, // Replace with your COM port
		Baud:        config.BaudRate,   // Set the baud rate (default is 9600 for CH340)
		ReadTimeout: time.Second * 2,   // Optional: Set a read timeout
		Parity:      serial.ParityNone, // Optional: Set the parity mode
	}

	port, err := serial.OpenPort(serialConfig)
	if err != nil {
		return nil, err
	}

	ld2451 := &LD2451{
		config:  config,
		targets: make(chan Target, config.TargetBufferSize),
		errors:  make(chan error),
		port:    port,
	}

	go ld2451.read()

	return ld2451, nil
}

func (ld2451 *LD2451) Close() {
	ld2451.port.Close()
}

func (ld2451 *LD2451) read() {
	for {
		buf := make([]byte, 1)
		_, err := ld2451.port.Read(buf)
		if err != nil {
			ld2451.errors <- err
			return
		}

		if buf[0] != 0xf4 {
			continue
		}

		//check if the next 3 bytes are the frame header
		buf = make([]byte, 3)
		_, err = ld2451.port.Read(buf)
		if err != nil {
			ld2451.errors <- err
			return
		}

		if bytes.Equal(buf, frameheader[1:]) {
			//get length of the frame (next 2 bytes)
			buf = make([]byte, 2)
			_, err := ld2451.port.Read(buf)
			if err != nil {
				ld2451.errors <- err
				return
			}
			frameLength := int(buf[1])<<8 | int(buf[0])
			if frameLength == 0 {
				//restart loop if there is no more data
				//read the next 4 bytes, this is the frame footer []byte{0xf8, 0xf7, 0xf6, 0xf5}
				buf = make([]byte, 4)
				_, err = ld2451.port.Read(buf)
				if err != nil {
					ld2451.errors <- err
					return
				}
				continue
			}
			//read the rest of the frame
			buf = make([]byte, frameLength)
			_, err = ld2451.port.Read(buf)
			if err != nil {
				ld2451.errors <- err
				return
			}
			//get the number of targets in the frame, this is the next byte after the frame length
			numTargets := int(buf[0])
			//move to the next byte
			buf = buf[1:]
			//loop over and parse each target
			for i := 0; i < numTargets; i++ {
				target := Target{}
				//get the target data
				target.Alarm = int(buf[0]) == 1
				target.Angle = int(buf[1]) - 0x80
				target.Distance = int(buf[2])
				target.Direction = Direction(buf[3])
				target.Speed = int(buf[4])
				target.SNR = int(buf[5])

				//send the target to the channel
				ld2451.targets <- target
				//move to the next target
				buf = buf[6:]
			}
			//flush the rest of the frame
			ld2451.port.Flush()
		}
	}
}

func (ld2451 *LD2451) ReadTarget() (Target, error) {
	select {
	case target := <-ld2451.targets:
		return target, nil
	case err := <-ld2451.errors:
		return Target{}, err
	}
}
