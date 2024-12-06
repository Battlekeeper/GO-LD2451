package LD2451

import (
	"bytes"
	"fmt"
	"time"

	"github.com/tarm/serial"
)

type Config struct {
	SerialPort       string
	BaudRate         int
	TargetBufferSize int //Size of the channel buffer to store targets in
}

type Target struct {
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
		Name:        config.SerialPort,
		Baud:        config.BaudRate,
		ReadTimeout: time.Second * 2,
		Parity:      serial.ParityNone,
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

	ld2451.syn()

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

		if buf[0] != frameheader[0] {
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
			//move to the next byte AND skip alarm state
			buf = buf[2:]

			//loop over and parse each target
			for i := 0; i < numTargets; i++ {
				target := Target{}
				//get the target data
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
			buf = make([]byte, 4)
			_, err = ld2451.port.Read(buf)
			if err != nil {
				ld2451.errors <- err
				return
			}
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

func (ld2451 *LD2451) sendCommand(command []byte) {
	//send bytes FD FC FB FA 04 00 FF 00 01 00 04 03 02 01
	ld2451.port.Write([]byte{0xfd, 0xfc, 0xfb, 0xfa, 0x04, 0x00, 0xff, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01})
	//read the response
	buf := make([]byte, 1)
	_, err := ld2451.port.Read(buf)
	if err != nil {
		ld2451.errors <- err
		return
	}
	if buf[0] != 0xfd {
		ld2451.errors <- fmt.Errorf("failed to send command to the LD2451")
		return
	}

	buf = make([]byte, 17)
	_, err = ld2451.port.Read(buf)
	if err != nil {
		ld2451.errors <- err
		return
	}
	status := buf[7:]
	status = status[:len(status)-8]
	endFrame := buf[len(buf)-4:]

	if !bytes.Equal(endFrame, []byte{0x04, 0x03, 0x02, 0x01}) || !bytes.Equal(status, []byte{00, 00}) {
		ld2451.errors <- fmt.Errorf("failed to send command to the LD2451")
		return
	}

	//send command

	ld2451.port.Write([]byte{0xfd, 0xfc, 0xfb, 0xfa, 0x02, 0x00, 0xfe, 0x00, 0x04, 0x03, 0x02, 0x01})

	buf = make([]byte, 1)
	_, err = ld2451.port.Read(buf)
	if err != nil {
		ld2451.errors <- err
		return
	}
	if buf[0] != 0xfd {
		ld2451.errors <- fmt.Errorf("failed to send command to the LD2451")
		return
	}

	buf = make([]byte, 13)
	_, err = ld2451.port.Read(buf)
	if err != nil {
		ld2451.errors <- err
		return
	}
	status = buf[7:]
	status = status[:len(status)-4]
	endFrame = buf[len(buf)-4:]

	if !bytes.Equal(endFrame, []byte{0x04, 0x03, 0x02, 0x01}) || !bytes.Equal(status, []byte{00, 00}) {
		ld2451.errors <- fmt.Errorf("failed to send command to the LD2451")
		return
	}
}
