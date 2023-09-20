package net.wavem.uvc.rms.common.jwt

import io.jsonwebtoken.Claims
import io.jsonwebtoken.Jwts
import io.jsonwebtoken.SignatureAlgorithm
import jakarta.annotation.PostConstruct
import org.springframework.beans.factory.annotation.Value
import org.springframework.stereotype.Component
import java.util.*


@Component
class JwtProvider {

    @Value("\${spring.jwt.secret}")
    private lateinit var secretKey: String

    private val accessTokenValidMillisecond = 60 * 60 * 1000L

    private val refreshTokenValidMillisecond = 14 * 24 * 60 * 60 * 1000L

    @PostConstruct
    protected fun secretInit() {
        secretKey = Base64.getEncoder().encodeToString(secretKey.toByteArray());
    }

    fun encode(key: String, value: String): String {
        val claims = Jwts.claims()
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

    fun getLoginId(token: String, secretKey: String): String {
        return extractClaims(token, secretKey)["loginId"].toString()
    }

    fun isExpired(token: String, secretKey: String): Boolean {
        val expiredDate: Date = extractClaims(token, secretKey).expiration
        return expiredDate.before(Date())
    }

    private fun extractClaims(token: String, secretKey: String): Claims {
        return Jwts.parser().setSigningKey(secretKey).parseClaimsJws(token).body
    }
}