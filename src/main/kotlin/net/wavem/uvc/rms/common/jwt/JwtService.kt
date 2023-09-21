package net.wavem.uvc.rms.common.jwt

import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import io.jsonwebtoken.Claims
import io.jsonwebtoken.Jwts
import io.jsonwebtoken.SignatureAlgorithm
import jakarta.annotation.PostConstruct
import org.springframework.beans.factory.annotation.Value
import org.springframework.stereotype.Service
import java.util.*


@Service
class JwtService {

    @Value("\${spring.jwt.secret}")
    private lateinit var secretKey: String

    private val accessTokenValidMillisecond = 60 * 60 * 1000L

    private val refreshTokenValidMillisecond = 14 * 24 * 60 * 60 * 1000L

    @PostConstruct
    protected fun secretInit() {
        secretKey = Base64.getEncoder().encodeToString(secretKey.toByteArray());
    }

    fun encode(key: String, value: String): String {
        val claims: Claims = Jwts.claims()
        claims[key] = value

        return Jwts.builder()
            .setClaims(claims)
            .setIssuedAt(Date(System.currentTimeMillis()))
            .setExpiration(Date(System.currentTimeMillis() + accessTokenValidMillisecond))
            .signWith(SignatureAlgorithm.HS256, secretKey)
            .compact()
    }


    fun decode(jwt: String): Claims {
        return Jwts.parser()
            .setSigningKey(secretKey)
            .parseClaimsJws(jwt)
            .body
    }
}

inline fun <reified T> convertClaimsToJsonObject(decodedClaims: Claims, key: String): T {
    val decodedClaimsString: String = decodedClaims[key].toString()

    val decodedClaimsJsonObject: JsonObject = JsonParser().parse(decodedClaimsString).asJsonObject
    return Gson().fromJson(decodedClaimsJsonObject, T::class.java)
}