# Curso de Sistemas Embarcados com STM32

## 📘 Visão Geral

Este curso tem como objetivo capacitar os alunos no desenvolvimento de sistemas embarcados - VIRTUS CC utilizando a plataforma STM32, amplamente empregada na indústria devido ao seu desempenho, flexibilidade e robustez. O curso cobre desde os conceitos fundamentais até aplicações práticas com sensores, atuadores e comunicação.

---

## 🎯 Objetivos

- Introduzir os conceitos de sistemas embarcados.
- Capacitar o aluno no uso da linha de microcontroladores STM32.
- Ensinar a configurar e utilizar periféricos como GPIO, UART, I2C, SPI, ADC, PWM, etc.
- Desenvolver aplicações reais utilizando a IDE STM32CubeIDE e HAL/LL drivers.
- Estimular boas práticas de programação embarcada.

---

## 🧾 Pré-requisitos

- Conhecimentos básicos de linguagem C.
- Noções de eletrônica digital.
- Familiaridade com IDEs e compiladores.

---

## 🧰 Ferramentas e Recursos

- **Placa de desenvolvimento**: STM32F767ZI ou STM32 Nucleo.
- **IDE**: STM32CubeIDE (gratuito - STMicroelectronics)
- **Drivers e bibliotecas**: STM32CubeMX, HAL/LL Drivers
- **Debug**: ST-Link V2 ou onboard debugger.
- **Outros**: Serial Terminal (Putty, Tera Term), Multímetro, Protoboard, Jumpers, Sensores (DHT11, MPU6050 etc.)

---

## 📚 Conteúdo Programático

### Aula 1 - Introdução a Sistemas Embarcados
- Definições e aplicações
- Arquiteturas de microcontroladores
- Família STM32 e arquitetura ARM Cortex-M

### Aula 2 - Ambiente de Desenvolvimento
- Instalação e configuração do STM32CubeIDE
- Introdução ao STM32CubeMX
- Compilação e gravação via ST-Link

### Aula 3 - Manipulação de GPIO
- Entrada e saída digital
- Piscar LED (Hello World)
- Leitura de botões

### Aula 4 - Interrupções e Timers
- NVIC e prioridades
- Delay com timer
- PWM para controle de brilho e motores

### Aula 5 - Conversão Analógico-Digital (ADC)
- Leitura de sensores analógicos
- Potenciômetro e sensor de temperatura

### Aula 6 - Comunicação Serial (USART)
- UART com terminal serial
- Comunicação com PC
- Enviar/receber strings

### Aula 7 - I2C e SPI
- Comunicação com sensores digitais
- Leitura de dados do MPU6050
- Display OLED com I2C

### Aula 8 - Projeto Final
- Escolha do projeto (Ex.: Estação meteorológica, robô seguidor de linha)
- Integração dos módulos
- Documentação do projeto

---

## 📎 Referências

- [Documentação oficial STM32](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [ARM Cortex-M Architecture](https://developer.arm.com/documentation)
- [Livro] Sistemas Embarcados - Luiz Affonso Guedes
- [Livro] The Definitive Guide to ARM Cortex-M3 - Joseph Yiu

---

> _"Engenharia não é apenas construir sistemas, mas criar soluções que interajam com o mundo real."_  
> — Autor desconhecido

