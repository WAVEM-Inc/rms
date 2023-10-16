plugins {
    kotlin("jvm") version "1.9.0"
    id("org.springframework.boot") version "3.1.3"
    id("io.spring.dependency-management") version "1.1.3"
    id("org.jetbrains.kotlin.plugin.spring") version "1.8.22"
}

group = "net.wavem.uvc"
version = "1.0-SNAPSHOT"

repositories {
    mavenCentral()
}

dependencies {
    implementation("io.github.lambdaprime:jros2client:1.0")
    implementation("io.github.pinorobotics:rtpstalk:4.0")
    implementation(kotlin("reflect"))
    implementation("io.reactivex:rxjava:1.3.8")
    implementation ("org.locationtech.proj4j:proj4j:1.2.2")
    implementation ("org.springframework.boot:spring-boot-starter-web")
    implementation ("org.springframework.boot:spring-boot-starter-thymeleaf")
    implementation ("org.springframework.boot:spring-boot-starter-integration")
    implementation ("org.springframework.integration:spring-integration-core")
    implementation ("org.springframework.integration:spring-integration-mqtt")
    implementation ("org.springframework.integration:spring-integration-stream")
    implementation ("io.jsonwebtoken:jjwt-api:0.11.2")
    implementation ("io.jsonwebtoken:jjwt-impl:0.11.2")
    implementation ("io.jsonwebtoken:jjwt-jackson:0.11.2")
    implementation ("org.jetbrains.kotlinx:kotlinx-serialization-json:1.3.1")
    implementation ("com.fasterxml.jackson.module:jackson-module-kotlin")
    implementation ("org.jetbrains.kotlin:kotlin-reflect")
    implementation ("org.springframework.boot:spring-boot-devtools")
    implementation ("com.google.code.gson:gson:2.8.9")
    implementation ("javax.xml.bind:jaxb-api:2.3.0")
    implementation ("org.glassfish.jaxb:jaxb-core:2.3.0.1")
    implementation ("com.sun.xml.bind:jaxb-impl:2.3.1")
    implementation ("javax.activation:activation:1.1.1")
    compileOnly ("org.projectlombok:lombok")
    annotationProcessor ("org.projectlombok:lombok")
    testImplementation ("org.springframework.boot:spring-boot-starter-test")
    testImplementation (kotlin("test"))
}

tasks.test {
    useJUnitPlatform()
}

kotlin {
    jvmToolchain(17)
}