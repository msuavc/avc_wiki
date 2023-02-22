FROM mcr.microsoft.com/dotnet/sdk:6.0 AS builder
WORKDIR /build
COPY . /build
RUN dotnet tool install retypeapp --tool-path /bin

FROM httpd:latest
CMD [ "/bin/bash" ]
